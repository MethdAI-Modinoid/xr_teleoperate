# Fork notes

This is a local fork of `unitreerobotics/xr_teleoperate`, cloned into the painting-pipeline
tree on 2026-09-05 so it can be modified for the headset-only recording backend described in
`../XR_SWAP_PLAN.md`.

## Provenance

Copied from `/mnt/ssd2/xr_teleoperate` (owned by another user on this machine, not writable
by `aryan`, which is why a local fork exists at all).

- parent HEAD: `7dc9aa1` "[update] gate ee control with XR motion data ready signal"
- `origin` repointed to `https://github.com/unitreerobotics/xr_teleoperate.git`
  (the source clone used an SSH remote, which needs keys we don't have here).

## Deliberate differences from the source clone

1. **The source clone's uncommitted Isaac Sim work-in-progress was NOT carried over.**
   `teleop/teleimager` had uncommitted edits there: reading frames from `/dev/shm` with
   Isaac Lab SHM headers, `isaacsim_enable=True` by default, an `ImageClient` default of
   `127.0.0.1:60001`, wrist cameras commented out, and `head_camera.type` flipped to
   `opencv`. Those are wrong defaults for a headset + RealSense station, and they were
   someone else's half-finished work.

   Nothing was lost: the full diff is saved at
   `patches/teleimager-isaacsim-wip-from-ssd2.patch` and can be re-applied with
   `git -C teleop/teleimager apply ../../patches/teleimager-isaacsim-wip-from-ssd2.patch`.

2. **`teleop/televuer/{cert,key}.pem` were regenerated.** `key.pem` is mode 600 and owned by
   the other user, so it could not be copied; a cert without its key is useless. The
   replacement is a fresh self-signed pair (verified as matching), now with a
   subjectAltName covering `localhost`, `127.0.0.1` and this machine's LAN IP -- the
   original had no SAN at all. These files are gitignored, so each station generates its
   own. To regenerate (substituting the station's own LAN IP):

       openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
           -keyout key.pem -out cert.pem \
           -subj "/C=IN/ST=dl/O=Internet Widgits Pty Ltd" \
           -addext "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:<LAN_IP>"

   Vuer serves WebXR over HTTPS because browsers require a secure context for it. The cert
   is self-signed, so the headset's browser will warn on first connect; that is expected.

3. **A stray 0-byte file `teleop/t` was deleted.**

4. ~~teleimager logger calls fixed by hand~~ **REVERTED** -- this was a real bug
   (`logging_mp.get_logger(__name__, level=...)` against logging-mp 0.2.4, which exports
   `getLogger` and takes no `level`), but upstream had already fixed it in the commit the
   parent records. Syncing the submodule was the correct fix; the hand-patch was not needed.
   Kept here as a reminder to check the recorded submodule commit before patching a
   submodule by hand.

5. **`teleop/teleimager/cam_config_client.yaml` rewritten as a true no-camera fallback.**
   ImageClient reads this ONLY when no image server answers — so it now describes exactly
   that situation: all cameras disabled. Left enabled (upstream's state, plus an Isaac Sim
   remnant `physical_path: /dev/shm/isaac_head_image_shm` inherited from the source clone),
   the record loop crashes as soon as recording starts: it does
   `colors["color_0"] = head_img.bgr[:, :W//2]` guarded only by `head_img is not None`, and
   with no publisher `head_img` is a frame object whose `.bgr` is None. `enable_webrtc` is
   deliberately left `true` so `xr_need_local_img` stays False — with both webrtc and
   pass-through off, a different unguarded `head_img.bgr` dereference is reached. Upstream
   kept as `cam_config_client.yaml.upstream`.

6. **`teleop/teleimager/cam_config_server.yaml` rewritten for a recording station** -- one
   head-mounted RealSense, `binocular: false`, wrist cameras disabled. Upstream's version
   (which describes the G1's own head/wrist rig) is kept alongside as
   `cam_config_server.yaml.upstream`.

## Local changes made for the headset-only backend

- `teleop/utils/episode_writer.py` -- `add_item` gained `head_pose=` and `timestamp=`;
  colour-frame encoding is now configurable (`color_format`, `jpeg_quality`, or the
  `XR_COLOR_FORMAT`/`XR_JPEG_QUALITY` env vars) and defaults to JPEG **q100** rather than
  OpenCV's lossy q95 default.
- `teleop/teleop_hand_and_arm.py` -- `--no-robot` flag; all `unitree_sdk2py` imports made
  lazy so a station can install without the Unitree SDK; records head_pose/timestamp.
- `teleop/robot_control/robot_arm_virtual.py` -- new, `G1_29_VirtualArmController`.
- `teleop/robot_control/robot_hand_virtual.py` -- new, `Dex3_1_VirtualController`.

See `../XR_SWAP_PLAN.md` (items A, B) and `../STATION_SETUP.md` (install recipe).

## Submodule state

**CORRECTED 2026-09-05.** An earlier version of this file claimed the source clone's
submodules were checked out *ahead* of what the parent records, and kept that state on the
theory that reverting might drop a fix. That was wrong in both directions: `c77e596` is an
ANCESTOR of the recorded `2aab15d`, and `948f65f` an ancestor of `766de45` -- the source
clone was simply STALE, not ahead. Preserving it broke the entry point at runtime:

    TypeError: ImageClient.__init__() got an unexpected keyword argument 'request_bgr'

because the parent's `teleop_hand_and_arm.py` expects the newer teleimager API.

All submodules are now at the commits the parent records (`git submodule update --init
--recursive`). Do not "preserve" a detached submodule state again without first checking
`git merge-base --is-ancestor` in both directions.

| submodule | recorded by parent = checked out |
|---|---|
| `teleop/robot_control/dex-retargeting` | `d7753d3` |
| `teleop/teleimager` | `2aab15d` |
| `teleop/televuer` | `766de45` |

That sync also **superseded a hand-patch**: item 4 below was fixing `logging_mp.get_logger`
by hand, but the recorded commit `2aab15d` ("[upgrade] logging_mp") already contains exactly
that fix. The hand-patch was reverted.
