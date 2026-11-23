#!/usr/bin/env python3
"""Join clocks only within a host, and stages by session/epoch/sequences."""
import argparse,csv,json,statistics,html,os
from collections import Counter
from pathlib import Path

def read(p):
 rows=[]
 for line in Path(p).read_text().splitlines():
  try: rows.append(json.loads(line))
  except json.JSONDecodeError: pass # a live file can have one incomplete final line
 return rows
def key(r,seq):return (r.get('session'),r.get('epoch'),r.get(seq))
def stats(xs):
 xs=sorted(x for x in xs if x is not None)
 if not xs:return {'n':0}
 def pct(p):
  i=(len(xs)-1)*p;lo=int(i);hi=min(lo+1,len(xs)-1)
  return xs[lo]+(xs[hi]-xs[lo])*(i-lo)
 return {'n':len(xs),'mean':statistics.mean(xs),'p50':pct(.5),'p95':pct(.95),'max':max(xs)}
def intervals(rows,field):
 previous={};out=[]
 for r in rows:
  k=(r.get('session'),r.get('epoch'));v=r.get(field)
  if v is None:continue
  if k in previous:out.append(v-previous[k])
  previous[k]=v
 return out
def write_csv(p,rows):
 fields=list(dict.fromkeys(k for r in rows for k in r))
 with p.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=fields or ['no_samples']);w.writeheader();w.writerows(rows)
def analyze(mac,gateway,out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True)
 m=read(mac);g=read(gateway)
 scans={key(r,'scan_seq'):r for r in g if r.get('event') in ('scan_callback','gt_callback')}
 states={key(r,'state_seq'):r for r in g if r.get('event')=='state_sent'}
 per=[r for r in m if r.get('kind')=='perception']
 modality='Simulator GT' if any(r.get('sensor')=='simulator_gt' for r in per) else 'LiDAR'
 ctl=[r for r in m if r.get('kind')=='controller']
 candidates=[r for r in g if r.get('event')=='candidate_published']
 commands=[r for r in g if r.get('event')=='command_received']
 frame_rows=[]
 for r in per:
  src=scans.get(key(r,'scan_seq'),{})
  frame_rows.append({'session':r['session'],'epoch':r['epoch'],'scan_seq':r['scan_seq'],'scan_sim_s':r['stamp'],
   'odom_sim_s':r['stamp']+r['odom_time_offset_s'],'scan_odom_delta_ms':r['odom_time_offset_s']*1000,
   'ubuntu_scan_callback_wall_s':src.get('wall'),'mac_received_wall_s':r.get('received_wall'),
   'mac_process_start_wall_s':r.get('process_start_wall'),'mac_process_end_wall_s':r['wall'],
   'inference_ms':r['process_wall_s']*1000,'mac_queue_ms':r['queue_wall_s']*1000,
   'detections':len(r['detections']),'complete_slots':all(v is not None for v in r['slots']),
   'observed_people':sum(v is not None and v['stamp']==r['stamp'] for v in r['slots']),
   'capacity_exceeded':r['capacity_exceeded'],'mac_frames_replaced':r.get('frames_replaced')})
 unique_commands={};first_responses={}
 for r in commands:
  first_responses.setdefault(key(r,'state_seq'),r)
  if r.get('accepted'):unique_commands.setdefault(key(r,'state_seq'),r)
 chain=[]
 for r in ctl:
  sk=key(r,'state_seq');src=states.get(sk,{});scan=scans.get(key(r,'scan_seq'),{});cmd=unique_commands.get(sk,{})
  received=first_responses.get(sk,{})
  pub=next((p for p in candidates if key(p,'state_seq')==sk and p.get('command_valid') and p['wall']>=cmd.get('wall',float('inf'))),{})
  response=r['response']
  chain.append({'session':r['session'],'epoch':r['epoch'],'state_seq':r['state_seq'],'scan_seq':r['scan_seq'],
   'scan_sim_s':r['scan_stamp'],'state_sim_s':r['stamp'],'action_sim_s':response.get('action_timestamp',response.get('timestamp')),
   'scan_state_age_ms':(r['stamp']-r['scan_stamp'])*1000,'rpc_ms':r['rpc_wall_s']*1000,
   'state_to_response_wall_ms':(received['wall']-src['wall'])*1000 if received and src else None,
   'scan_to_response_wall_ms':(received['wall']-scan['wall'])*1000 if received and scan else None,
   'first_response_rejection_reason':received.get('rejection_reason'),
   'state_roundtrip_ms':(cmd['wall']-src['wall'])*1000 if cmd and src else None,
   'scan_to_first_candidate_wall_ms':(pub['wall']-scan['wall'])*1000 if pub and scan else None,
   'scan_to_first_candidate_sim_ms':(pub['sim_s']-scan['stamp'])*1000 if pub and scan else None,
   'accepted':bool(cmd),'published_valid':bool(pub),'control_updated':response.get('control_updated'),
   'output_source':response.get('output_source'),'v':response.get('action',{}).get('v'),'omega':response.get('action',{}).get('r')})
  chain[-1].update(observed_humans=response.get('observed_humans',len(r.get('slot_ids',[]))),mpc_feasible=response.get('mpc_feasible'),prediction_mode=response.get('prediction_mode'),jmid_ms=response.get('jmid_ms'))
  chain[-1].update(native_mpc=bool(response.get('selection')),jmid_deadline_status=response.get('jmid_deadline_status'),jmid_wait_ms=response.get('jmid_wait_ms'),jmid_completed_late_ms=response.get('jmid_completed_late_ms'),controller_prediction_source=response.get('controller_prediction_source'))
 # Deduplicate action timestamps: heartbeat/held-command output is not a fresh plan.
 fresh=[];seen=set()
 for r in commands:
  k=(r.get('session'),r.get('epoch'),r.get('action_stamp'))
  if r.get('accepted') and k not in seen:seen.add(k);fresh.append(r)
 computed_fresh=[r for r in fresh if r.get('packet',{}).get('output_source') not in (None,'history_warmup')]
 computed_chain=[r for r in chain if r['output_source'] not in (None,'history_warmup')]
 def duplicate_count(rows,field):
  counts=Counter((r.get('session'),r.get('epoch'),r.get(field)) for r in rows)
  return sum(v-1 for v in counts.values())
 def backsteps(rows,field):return sum(x<0 for x in intervals(rows,field))
 sync=[abs(r['scan_odom_delta_ms']) for r in frame_rows]
 rpc=[r['rpc_ms'] for r in chain]
 wall_intervals=intervals(fresh,'wall');sim_intervals=intervals(fresh,'action_stamp')
 summary={'clock_policy':'ROS simulation stamps share one /clock; elapsed latencies use same-host monotonic only. Unix clocks are descriptive, not used for latency.',
  'perception_source':modality,
  'mac_events':dict(Counter(r.get('kind') for r in m)), 'gateway_events':dict(Counter(r.get('event') for r in g)),
  'frames':len(frame_rows),'complete_slot_frames':sum(r['complete_slots'] and not r['capacity_exceeded'] for r in frame_rows),
  'duplicate_scan_callback_timestamps':duplicate_count(list(scans.values()),'stamp'),
  'duplicate_perception_timestamps':duplicate_count(per,'stamp'),
  'scan_time_backsteps_within_epoch':backsteps(list(scans.values()),'stamp'),
  'perception_time_backsteps_within_epoch':backsteps(per,'stamp'),
  'controller_output_sources':dict(Counter(r['output_source'] for r in chain)),
  'controller_observed_count_histogram':dict(Counter(r['observed_humans'] for r in chain)),
  'mpc_feasible_responses':sum(bool(r['mpc_feasible']) for r in chain),
  'native_mpc_including_deadline_responses':sum(r['native_mpc'] for r in chain),
  'native_mpc_including_deadline_rpc_ms':stats([r['rpc_ms'] for r in chain if r['native_mpc']]),
  'jmid_deadline_status_counts':dict(Counter(r['jmid_deadline_status'] for r in chain if r['jmid_deadline_status'] is not None)),
  'native_jmid_mpc_responses':sum(r['jmid_ms'] is not None for r in chain),
  'native_jmid_mpc_rpc_ms':stats([r['rpc_ms'] for r in chain if r['jmid_ms'] is not None]),
  'local_plan_responses':sum(r['prediction_mode'] in ('no_observed_people','observed_people_warmup') for r in chain),
  'warmup_responses':sum(r['output_source']=='history_warmup' for r in chain),
  'detections_histogram':dict(Counter(r['detections'] for r in frame_rows)),
  'scan_odom_abs_ms':stats(sync),'scan_odom_within_35ms':bool(sync) and max(sync)<=35.000001,
  'scan_sim_interval_s':stats(intervals(list(scans.values()),'stamp')),
  'scan_callback_wall_interval_s':stats(intervals(list(scans.values()),'wall')),
  'inference_ms':stats([r['inference_ms'] for r in frame_rows]),'mac_queue_ms':stats([r['mac_queue_ms'] for r in frame_rows]),
  'controller_rpc_ms':stats(rpc),'controller_rpc_over_250ms':sum(x>250 for x in rpc),
  'computed_controller_rpc_ms':stats([r['rpc_ms'] for r in computed_chain]),
  'controller_request_sim_interval_s':stats(intervals([r for r in m if r.get('kind')=='controller_request'],'stamp')),
  'fresh_accepted_actions':len(fresh),'fresh_action_wall_interval_s':stats(wall_intervals),'fresh_action_sim_interval_s':stats(sim_intervals),
  'fresh_action_sim_gap_over_0_30s':sum(x>.300001 for x in sim_intervals),
  'computed_fresh_accepted_actions':len(computed_fresh),
  'computed_fresh_action_wall_interval_s':stats(intervals(computed_fresh,'wall')),
  'computed_fresh_action_sim_interval_s':stats(intervals(computed_fresh,'action_stamp')),
  'computed_scan_to_candidate_wall_ms':stats([r['scan_to_first_candidate_wall_ms'] for r in computed_chain]),
  'computed_scan_to_candidate_sim_ms':stats([r['scan_to_first_candidate_sim_ms'] for r in computed_chain]),
  'computed_state_to_response_wall_ms':stats([r['state_to_response_wall_ms'] for r in computed_chain]),
  'computed_scan_to_response_wall_ms':stats([r['scan_to_response_wall_ms'] for r in computed_chain]),
  'computed_250ms_rpc_budget_met':None if not computed_chain else max(r['rpc_ms'] for r in computed_chain)<=250,
  'state_roundtrip_ms':stats([r['state_roundtrip_ms'] for r in chain]),
  'scan_to_candidate_wall_ms':stats([r['scan_to_first_candidate_wall_ms'] for r in chain]),
  'scan_to_candidate_sim_ms':stats([r['scan_to_first_candidate_sim_ms'] for r in chain]),
  'candidate_wall_interval_s':stats(intervals(candidates,'wall')),
  'candidate_sim_interval_s':stats(intervals(candidates,'sim_s')),
  'invalid_candidate_nonzero':sum(not r['command_valid'] and (r['v']!=0 or r['omega']!=0) for r in candidates),
  'command_rejected_messages':sum(not r.get('accepted') for r in commands),
  'command_rejection_reasons':dict(Counter(r.get('rejection_reason','not_instrumented') for r in commands if not r.get('accepted'))),
  'halt_reasons':dict(Counter(r.get('reason') for r in g if r.get('event')=='halt_received')),
  'first_scan_sim_s':min((r['stamp'] for r in scans.values()),default=None),'last_scan_sim_s':max((r['stamp'] for r in scans.values()),default=None),
  'observed_scan_wall_span_s':max((r['wall'] for r in scans.values()),default=0)-min((r['wall'] for r in scans.values()),default=0)}
 write_csv(out/'frames.csv',frame_rows);write_csv(out/'control_timeline.csv',chain)
 write_csv(out/'candidate_timeline.csv',[{k:r.get(k) for k in ('session','epoch','wall','sim_s','state_seq','scan_seq','action_stamp','command_valid','v','omega','drive')} for r in candidates])
 (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
 # Static scientific timing plots, suitable for exporting alongside the CSV data.
 os.environ.setdefault('MPLCONFIGDIR',str(out.resolve()/'mpl-cache'))
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,ax=plt.subplots(5,1,figsize=(12,14),layout='constrained',sharex=True)
 ax[0].plot([r['scan_sim_s'] for r in frame_rows],[r['scan_odom_delta_ms'] for r in frame_rows],'.',ms=2);ax[0].axhline(35,c='r',ls='--');ax[0].axhline(-35,c='r',ls='--');ax[0].set_ylabel('Odom - scan (ms)')
 ax[1].plot([r['scan_sim_s'] for r in frame_rows],[r['inference_ms'] for r in frame_rows],'.',ms=2,label='MPS inference');ax[1].plot([r['state_sim_s'] for r in chain],[r['rpc_ms'] for r in chain],'.',ms=2,label='controller RPC');ax[1].axhline(250,c='r',ls='--',label='250 ms');ax[1].legend();ax[1].set_ylabel('Wall duration (ms)')
 ax[2].plot([r['state_sim_s'] for r in computed_chain],[r['scan_to_response_wall_ms'] for r in computed_chain],'.',ms=3,label='scan to response (accepted or rejected)')
 ax[2].plot([r['state_sim_s'] for r in computed_chain],[r['scan_to_first_candidate_wall_ms'] for r in computed_chain],'.',ms=4,label='scan to accepted candidate')
 ax[2].set_ylabel('End-to-end (ms)');ax[2].legend()
 if not any(r['published_valid'] for r in computed_chain):ax[2].text(.02,.85,'No computed action reached a valid candidate; warmup excluded',transform=ax[2].transAxes)
 reqs=[r for r in m if r.get('kind')=='controller_request']
 gaps=intervals(reqs,'stamp')
 if len(gaps)==max(0,len(reqs)-1):ax[3].plot([r['stamp'] for r in reqs[1:]],gaps,'.',label='request simulation interval')
 ax[3].axhline(.25,color='r',ls='--',label='0.25 s target');ax[3].set_ylabel('Request gap (s)');ax[3].legend()
 ax[4].plot([r['sim_s'] for r in candidates],[r['v'] for r in candidates],label='candidate v (m/s)');ax[4].plot([r['sim_s'] for r in candidates],[int(r['command_valid']) for r in candidates],alpha=.5,label='valid command (includes warmup zero)');ax[4].legend();ax[4].set_ylabel('Output');ax[4].set_xlabel('ROS simulation time (s)')
 for a in ax:a.grid(alpha=.2)
 mode='drive' if any(r.get('drive') for r in candidates) else 'shadow'
 fig.suptitle(f'Live {modality} / JMID-MPC timing — {mode} mode')
 fig.savefig(out/'timing.png',dpi=150);plt.close(fig)
 def fmt(metric,field):
  value=summary[metric].get(field)
  return '표본 없음' if value is None else f'{value:.2f}'
 overview='<h2>판정에 필요한 수치</h2><table border="1" cellpadding="9" style="border-collapse:collapse"><tr><th>항목</th><th>측정</th></tr>'
 for label,value in [
  ('처리 프레임',str(len(per))),('scan/odom 차이 최대',fmt('scan_odom_abs_ms','max')+' ms (허용 35 ms)'),
  ('중복 처리 / 시각 역행',f"{summary['duplicate_perception_timestamps']} / {summary['perception_time_backsteps_within_epoch']} 건"),
  ('MPS 추론 p95',fmt('inference_ms','p95')+' ms'),
  ('준비 대기를 제외한 제어 RPC p95',fmt('computed_controller_rpc_ms','p95')+' ms (목표 250 ms)'),
  ('새 계산 결과가 승인된 수',str(len(computed_fresh))),
  ('거부 사유',str(summary['command_rejection_reasons'])),
  ('무효 상태에서 비영(非零) 후보 출력',str(summary['invalid_candidate_nonzero'])+' 건')]:
  overview+='<tr><td>'+html.escape(label)+'</td><td>'+html.escape(value)+'</td></tr>'
 overview+='</table><p>새 계산 결과가 승인된 수가 0이면 end-to-end 유효 제어 출력은 검증되지 않았습니다. 준비 대기 0속도 수신을 제어 성공으로 세지 않습니다.</p>'
 page='<html lang="ko"><meta charset="utf-8"><title>실행 시간축 검증</title><style>body{font:16px system-ui;max-width:1100px;margin:36px auto;padding:0 20px}img{width:100%}pre{background:#f3f5f7;padding:20px;overflow:auto}</style><h1>LiDAR 실행 시간축 검증</h1><p>실제 Gazebo LiDAR 입력과 Mac MPS 처리 결과. Shadow 모드이며 로봇 구동 검증과 구분합니다. 20 Hz 후보 메시지 발행은 새 제어 결과 4 Hz를 의미하지 않습니다. 누락 데이터는 성공으로 판정하지 않습니다.</p>'+overview+'<img src="timing.png"><h2>측정 요약</h2><pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre><p><a href="frames.csv">프레임 CSV</a> · <a href="control_timeline.csv">제어 시간축 CSV</a> · <a href="candidate_timeline.csv">출력 CSV</a></p></html>'
 (out/'report.html').write_text(page)
 if modality=='Simulator GT':
  (out/'report.html').write_text(page.replace('실제 Gazebo LiDAR 입력과 Mac MPS 처리 결과.','실제 Gazebo /crowd_sim/states 권위 상태를 입력한 GT 모드. DR-SPAAM 감지를 사용하지 않으며 JMID는 MPS를 사용합니다.').replace('LiDAR 실행 시간축 검증','GT 실행 시간축 검증'))
 return summary
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--mac',required=True);p.add_argument('--gateway',required=True);p.add_argument('--out',required=True);a=p.parse_args();print(json.dumps(analyze(a.mac,a.gateway,a.out),indent=2,ensure_ascii=False))
