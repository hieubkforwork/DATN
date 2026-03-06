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
### Linux / WSL environment recommended

**Step 1: Open PowerShell as Administrator**
* Click Start Menu
* Search for PowerShell
* Right-click Windows PowerShell
* Select Run as Administrator

**Step 2: Install WSL**
Run the following command:
```
wsl --install
```

This command will:
* Enable WSL
* Enable the Virtual Machine Platform
* Install the default Linux distribution (Ubuntu)

**Step 3: Restart the Computer**
After installation finishes, restart the system to complete the setup.

**Step 4: Initialize Ubuntu**
After restarting:
* Open Ubuntu from the Start Menu
* Create a Linux username and password when prompted
```
Example:
Username: developer
Password: ********
```
**Step 5: Verify WSL Installation**
Run the following command in PowerShell or Ubuntu terminal:
```
wsl -l -v
```
Expected output example:
```
NAME      STATE           VERSION
Ubuntu    Running         2
```
This indicates that WSL version 2 is successfully installed.

### Docker Desktop installed
Installing Docker Desktop
Step 1: Download Docker Desktop
Download Docker Desktop from the official website:
```
https://www.docker.com/products/docker-desktop/
```
Select Docker Desktop for Windows.

Step 2: Install Docker Desktop
Run the downloaded installer
During installation, ensure the following option is selected:
```
Use WSL 2 instead of Hyper-V
```
Click Install

Step 3: Restart the Computer
Restart your computer after installation.

### Enable Docker Integration with WSL

Open **Docker Desktop**
Go to **Settings**
Navigate to:
```
Resources → WSL Integration
```
Enable:
```
Enable integration with my default WSL distro
Ubuntu
```
Click **Apply & Restart**

### Git installed
.git

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

