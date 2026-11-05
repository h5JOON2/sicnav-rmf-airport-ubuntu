#!/usr/bin/env bash
# Ubuntu 24.04 / ROS2 Jazzy, with Gazebo/RMF already installed.
set -euo pipefail
if [ "$(uname -s)" != Linux ]; then
    echo 'Run this setup inside Ubuntu, not on the Mac host.' >&2
    exit 1
fi
SICNAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SICNAV_WORKSPACE="$(cd "$SICNAV_ROOT/../.." && pwd)"
SICNAV_PYTHON="$SICNAV_WORKSPACE/.venv/bin/python"
for tool in cmake gcc g++ make curl colcon git; do
    command -v "$tool" >/dev/null || { echo "Missing build tool: $tool" >&2; exit 1; }
done
# Use ROS's system interpreter rather than a different conda Python.
/usr/bin/python3 -m venv --system-site-packages --without-pip "$SICNAV_WORKSPACE/.venv"
if ! "$SICNAV_PYTHON" -m pip --version >/dev/null 2>&1; then
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "$SICNAV_WORKSPACE/get-pip.py"
    "$SICNAV_PYTHON" "$SICNAV_WORKSPACE/get-pip.py"
fi
"$SICNAV_PYTHON" -m pip install -r "$SICNAV_ROOT/sicnav/requirements-ubuntu.txt"
# The Acados source tree is excluded from this repository; obtain the tested version.
if [ ! -f "$SICNAV_ROOT/acados/CMakeLists.txt" ]; then
    git clone --no-checkout https://github.com/acados/acados.git "$SICNAV_ROOT/acados"
    git -C "$SICNAV_ROOT/acados" checkout 285d382b6c6d59c0983644caf6ed2924ed2153cb
    git -C "$SICNAV_ROOT/acados" submodule update --init --recursive
fi
cmake -S "$SICNAV_ROOT/acados" -B "$SICNAV_WORKSPACE/native-build/acados" \
    -DACADOS_INSTALL_DIR="$SICNAV_ROOT/acados" -DBLASFEO_TARGET=GENERIC \
    -DHPIPM_TARGET=GENERIC -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=OFF
cmake --build "$SICNAV_WORKSPACE/native-build/acados" -j2
cmake --install "$SICNAV_WORKSPACE/native-build/acados"
if ! command -v cargo >/dev/null && [ ! -x "$HOME/.cargo/bin/cargo" ]; then
    curl --proto '=https' --tlsv1.2 -fsSL https://sh.rustup.rs -o "$SICNAV_WORKSPACE/rustup-init.sh"
    sh "$SICNAV_WORKSPACE/rustup-init.sh" -y --profile minimal --no-modify-path
fi
SICNAV_CARGO="$(command -v cargo || true)"
SICNAV_CARGO="${SICNAV_CARGO:-$HOME/.cargo/bin/cargo}"
"$SICNAV_CARGO" build --release --manifest-path "$SICNAV_ROOT/acados/interfaces/acados_template/tera_renderer/Cargo.toml"
cp "$SICNAV_ROOT/acados/interfaces/acados_template/tera_renderer/target/release/t_renderer" "$SICNAV_ROOT/acados/bin/t_renderer"
(cd "$SICNAV_ROOT/Python-RVO2"; "$SICNAV_PYTHON" -m pip install --no-build-isolation .)
"$SICNAV_PYTHON" -m pip install --no-build-isolation -e "$SICNAV_ROOT/acados/interfaces/acados_template"
# ROS setup scripts can reference unset optional environment variables.
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/sicnav_airport_ws/install/setup.bash" ]; then
    source "$HOME/sicnav_airport_ws/install/setup.bash"
fi
cd "$SICNAV_WORKSPACE"
colcon build --symlink-install --paths "$SICNAV_ROOT" --packages-select sicnav_airport_bringup
printf '\nSetup complete. Run: %s/scripts/run_ubuntu.sh\n' "$SICNAV_ROOT"
