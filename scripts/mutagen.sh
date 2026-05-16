mutagen sync create \
    --name micup-to-ubuntu \
    --sync-mode one-way-safe \
    --ignore-vcs \
    --ignore "cyberdog_ws" \
    --ignore "cyberdog_sim" \
    --ignore "eigen-git-mirror" \
    --ignore "lcm" \
    --ignore "__pycache__" \
    --ignore ".DS_Store" \
    --ignore "*.pyc" \
    /Users/phoen1xcode/Projects/MiCup/MiCup \
    phoen1xcode@192.168.0.26:/home/phoen1xcode/MiCup

# Usage:
# mutagen sync list
# mutagen sync monitor micup-to-ubuntu
# mutagen sync flush micup-to-ubuntu
# mutagen sync resume micup-to-ubuntu
# mutagen sync pause micup-to-ubuntu
# mutagen sync terminate micup-to-ubuntu
