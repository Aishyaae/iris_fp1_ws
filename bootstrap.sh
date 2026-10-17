#!/bin/bash

PACKAGES=(master vision communication)

usage() {
    cat <<EOF
Usage: $0 [-f|--force] [-h|--help]

Create the ROS 2 packages: ${PACKAGES[*]}

Options:
  -f, --force   Remove existing packages and create them again
  -h, --help    Show this help and exit

Without --force, the script exits if any package folder already exists.
EOF
}

FORCE=0
case "$1" in
    "")
        ;;
    -f|--force)
        FORCE=1
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 1
        ;;
esac

if [[ $FORCE -eq 1 ]]; then
    for pkg in "${PACKAGES[@]}"; do
        rm -rf "$pkg"
    done
else
    for pkg in "${PACKAGES[@]}"; do
        if [[ -d "$pkg" ]]; then
            echo "Package '$pkg' already exists. Use -f/--force to reset packages." >&2
            exit 1
        fi
    done
fi

for pkg in "${PACKAGES[@]}"; do
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 "$pkg"
done

git add -A -- "${PACKAGES[@]}"
if git diff --cached --quiet -- "${PACKAGES[@]}"; then
    echo "Nothing to commit."
else
    git commit -m "feat: initialize workspace & packages"
fi
