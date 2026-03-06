# Yocto Docker Build Environment

This project provides a simple environment to build Yocto images using Docker.

The repository contains scripts and configuration to automatically clone Poky and build a Docker container for the Yocto build system.

---

## Project Structure

```
yocto-project
│
├── Dockerfile          # Docker environment for Yocto build
├── README.md           # Project documentation
│
├── scripts
│   └── setup.sh        # Clone poky and build docker image
│
├── conf                # Yocto configuration templates
│
└── layers              # Custom project layers
```

---

## Requirements

Before using this project make sure you have:

* Docker Desktop installed
* Git installed
* Linux / WSL environment recommended

Test Docker installation:

```
docker run hello-world
```

---

## Setup Environment

Clone the repository:

```
git clone <repository-url>
cd yocto-project
```

Run the setup script:

```
./scripts/setup.sh
```

The script will:

1. Clone the Poky repository
2. Checkout the `kirkstone` branch
3. Build the Docker image `yocto-builder`

---

## Run Docker Container

After building the image run:

```
docker run -it --rm \
-v $(pwd)/..:/workspace \
-w /workspace \
yocto-builder bash
```

You will enter the container environment.

---

## Build Yocto Image

Inside the container:

```
cd poky
source oe-init-build-env
bitbake core-image-minimal
```

The build output will be located in:

```
build/tmp/deploy/images/
```

---

## Notes

* The `build/` directory should not be committed to Git.
* Only project layers and configuration files should be version controlled.
* Poky is cloned automatically by the setup script.

---

## Future Improvements

* Add custom layer `meta-myproject`
* Add build automation scripts
* Add CI/CD pipeline for Yocto builds

