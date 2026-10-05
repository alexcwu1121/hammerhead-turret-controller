# hammerhead-turret-controller

To build:
1. Install Podman >4.x
2. Install VSCode
3. Install VSCode Dev Containers extension
4. In Dev Containers settings, set the following:
    - Docker Path: `podman`
    - Docker Compose Path: `podman-compose`
    - Docker Socket: `/run/podman/podman.sock`

To quick-run pre-commit, builds, tests:
`podman-compose run --build pre-commit`
`podman-compose run --build build-debug`
`podman-compose run --build build-release`

CAN ID claims:
- Pub: 0x300 - 0x3FF
- Sub: 0x400 - 0x4FF

`arm-none-eabi-objcopy -O binary htc.elf htc.bin`
