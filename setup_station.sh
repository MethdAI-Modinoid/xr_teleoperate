#!/usr/bin/env bash
# Finish setting up a recording station after `conda env create -f environment.yml`.
#
# Handles the three things environment.yml cannot express -- see the comments there.
# Idempotent: safe to re-run.
#
#   conda activate xr_station && ./setup_station.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${CONDA_PREFIX:?activate the xr_station env first}/bin/python"
PIP="$CONDA_PREFIX/bin/pip"

echo "==> using $PY"
"$PY" -c "import pinocchio, pinocchio.casadi; print('    conda pinocchio', pinocchio.__version__, '+ casadi bindings OK')"

echo "==> torch (CPU build, from the PyTorch index)"
"$PIP" install -q "torch==2.3.0" --index-url https://download.pytorch.org/whl/cpu

# --no-deps is LOAD-BEARING here: dex-retargeting declares pin>=2.7.0, and letting pip
# satisfy that installs the PyPI `pin` wheel over conda's pinocchio, which has no casadi
# bindings. Its real deps are already in environment.yml.
echo "==> dex-retargeting (editable, --no-deps)"
"$PIP" install -q --no-deps -e "$REPO/teleop/robot_control/dex-retargeting"

echo "==> televuer + teleimager (editable)"
"$PIP" install -q -e "$REPO/teleop/televuer"
"$PIP" install -q -e "$REPO/teleop/teleimager"

# Both packages look for their own cert.pem/key.pem in their own directory. teleimager
# without them dies with a bare "[Errno 2] No such file or directory" in its WebRTC thread
# and then shuts the whole image server down a few seconds later.
echo "==> self-signed certs for televuer + teleimager"
LANIP="$(hostname -I | tr ' ' '\n' | grep -E '^(192|10|172)\.' | head -1 || true)"
SAN="DNS:localhost,IP:127.0.0.1"
[ -n "$LANIP" ] && SAN="$SAN,IP:$LANIP"
for d in "$REPO/teleop/televuer" "$REPO/teleop/teleimager"; do
    if [ ! -f "$d/key.pem" ]; then
        openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
            -keyout "$d/key.pem" -out "$d/cert.pem" \
            -subj "/C=IN/ST=dl/O=Internet Widgits Pty Ltd" \
            -addext "subjectAltName=$SAN" 2>/dev/null
        chmod 600 "$d/key.pem"
        echo "    generated $(basename "$d")/{cert,key}.pem  (SAN $SAN)"
    else
        echo "    $(basename "$d")/key.pem already present, left alone"
    fi
done

echo "==> verifying"
"$PY" - <<'PYCHECK'
import pinocchio, pinocchio.casadi, torch, dex_retargeting, televuer, teleimager
from teleimager.image_client import ImageClient
from televuer import TeleVuerWrapper
import inspect
assert "request_bgr" in inspect.signature(ImageClient.__init__).parameters, \
    "teleimager is behind the parent repo -- run: git submodule update --init --recursive"
import params_proto; assert hasattr(params_proto, "Flag"), "params-proto must be 2.x for vuer 0.0.60"
print("    all imports OK; pinocchio", pinocchio.__version__, "| torch", torch.__version__)
PYCHECK

cat <<'NEXT'

==> done. Remaining steps are hardware-specific:

  1. RealSense udev rules (librealsense reports 0 devices without them, even though the
     camera enumerates as UVC and a browser can open it):
         sudo cp patches/99-realsense-libusb.rules /etc/udev/rules.d/
         sudo udevadm control --reload-rules && sudo udevadm trigger
     then replug the camera.

  2. Camera MUST be on a USB3 link. Verify:
         python -c "import pyrealsense2 as rs; d=list(rs.context().query_devices())[0]; \
             print('USB', d.get_info(rs.camera_info.usb_type_descriptor))"
     Wants 3.2, not 2.1. On USB2 the D435i enumerates and configures but never delivers
     frames -- proven at 424x240@6 (~1% of USB2 bandwidth), so it is link integrity, not
     bandwidth. Use the cable that shipped with the camera.

  3. Put the camera serial in teleop/teleimager/cam_config_server.yaml (get it from
     `teleimager-server --cf --rs`), keep binocular: false for a single camera, then:
         teleimager-server --rs

  4. Record (see ../RUNBOOK.md):
         cd teleop && python teleop_hand_and_arm.py --no-robot --arm G1_29 --ee dex3 \
             --record --headless --task-dir ./utils/data --task-name my_task \
             --img-server-ip 127.0.0.1

  5. Confirm tracking is real before recording a lot: per-joint motion std must be NONZERO.
     Exactly 0.0 means televuer never received XR data. Set XR_TRACE_HANDS=1 to see why.
NEXT
