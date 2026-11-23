#!/bin/zsh
set -eu
exec ssh -o ExitOnForwardFailure=yes -N -R 127.0.0.1:9151:127.0.0.1:9151 -R 127.0.0.1:9152:127.0.0.1:9152 -R 127.0.0.1:9153:127.0.0.1:9153 "${1:?Usage: start_tunnel.command user@ubuntu-host}"
