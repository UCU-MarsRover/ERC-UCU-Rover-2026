<h1 align="center">Indomitus — Mars Rover Core</h1>

<p align="center">
  Everything that runs on <b>Indomitus</b> &mdash; the Mars rover built by the
  <a href="https://indomitus-ucu.vercel.app/">UCU Space Robotics</a> team at the
  Ukrainian Catholic University for the
  <a href="https://roverchallenge.eu/">European Rover Challenge</a>.
</p>

<p align="center">
  <a href="https://docs.ros.org/en/humble/"><img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white"></a>
  <a href="https://releases.ubuntu.com/22.04/"><img alt="Ubuntu 22.04" src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white"></a>
  <img alt="Platform" src="https://img.shields.io/badge/platform-amd64%20%7C%20arm64-informational">
</p>

<p align="center">
  <a href="https://github.com/UCUSpaceRobotics/indomitus-rover-core/actions/workflows/publish_image.yaml?query=branch%3Amain"><img alt="main build" src="https://img.shields.io/github/actions/workflow/status/UCUSpaceRobotics/indomitus-rover-core/publish_image.yaml?branch=main&label=main%20build&logo=github"></a>
  <a href="https://github.com/UCUSpaceRobotics/indomitus-rover-core/actions/workflows/publish_image.yaml?query=branch%3Adevelop"><img alt="develop build" src="https://img.shields.io/github/actions/workflow/status/UCUSpaceRobotics/indomitus-rover-core/publish_image.yaml?branch=develop&label=develop%20build&logo=github"></a>
  <a href="https://github.com/UCUSpaceRobotics/indomitus-rover-core/releases/latest"><img alt="main release" src="https://img.shields.io/github/v/release/UCUSpaceRobotics/indomitus-rover-core?label=main&color=green"></a>
  <a href="https://github.com/UCUSpaceRobotics/indomitus-rover-core/releases"><img alt="develop release" src="https://img.shields.io/github/v/release/UCUSpaceRobotics/indomitus-rover-core?include_prereleases&label=develop"></a>
</p>

---


## Table of Contents

- [Getting Started](#getting-started)
- [Documentation](#documentation)
- [Rover Usage](#rover-usage)
  - [Starting the Jetson-Controlled Rover](#starting-the-jetson-controlled-rover)
  - [Starting the Laptop-Controlled Rover](#starting-the-laptop-controlled-rover)
  - [Turning Off the Rover](#turning-off-the-rover)
  - [Current System Credentials and Network Info](#current-system-credentials-and-network-info)
  - [SSH Access to the Jetson](#ssh-access-to-the-jetson)
    - [SSH via Hotspot](#ssh-via-hotspot)
    - [SSH via Ethernet](#ssh-via-ethernet)
- [ROS\_DOMAIN\_ID](#ros_domain_id)
  - [Managing ROS\_DOMAIN\_ID](#managing-ros_domain_id)
- [Docker](#docker)
  - [Getting the Docker Image](#getting-the-docker-image)
  - [Start the Container and Build the Workspace](#start-the-container-and-build-the-workspace)
  - [Image Tags](#image-tags)
  - [Aliases](#aliases)
- [Scripts](#scripts)
  - [Deployment Script](#deployment-script)
  - [Script to Enter Containers](#script-to-enter-containers)
- [Testing](#testing)


---


## Getting Started
1. Read [README](README.md)
2. Set up [Docker](docker/README.md)
3. Read [CONTRIBUTING](https://github.com/UCUSpaceRobotics/.github/blob/main/.github/CONTRIBUTING.md) before making any changes


---


## Documentation
This file provides all necessary documentation for basic work with the project. For more details, you may refer to the other docs:

- [Hardware](docs/hardware/can_bus.md) — CAN bus wiring
- [Networking](docs/networking/ssh.md) — SSH connection
- [Software](docs/software/) — launch, nodes, testing, gazebo
- [Arm teleop](docs/arm/arm_teleop.md) — keyboard / gamepad Cartesian Servo
- [Arm sim](docs/arm/arm_sim.md) — RViz / MoveIt visualization, arm-only launch modes
- [Panel Simulation](docs/panel/panel_sim.md) — switch panel launch modes, combined rover+arm+panel sim
- [Testing](docs/software/testing.md) — code testing and test creation
- [Scripts](docs/scripts/) — scripts automating workflows
- [Deployment](docs/deployment.md) — code deployment to the rover
- [CAD](docs/CAD/README.md) — CAD files naming conventions


---


## Rover Usage

This section describes how to start the rover when it is controlled by the Jetson and the laptop.

### Starting the Jetson-Controlled Rover

Follow the steps described in the [QUICKSTART](QUICKSTART.md) to power on and operate the rover.

### Starting the Laptop-Controlled Rover

Follow these steps to power on and operate the rover using a laptop:

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button.

2. **Set up the CAN-to-USB adapter:** Connect the CAN-to-USB adapter to your laptop, then run the following commands in your terminal:

   Verify the CAN interface is visible to the laptop:
   ```bash
   ip link show can0
   ```
   *The output should display the `can0` interface details and include the state `DOWN`.*

   Bring the CAN interface up:
   ```bash
   ./scripts/host_setup.sh local --can
   ```

   Verify the interface is up:
   ```bash
   ip link show can0
   ```
   *The output should include the state `UP LOWER_UP`, and the blue LED on the adapter should light up.*

3. **Get the image and start the container on the laptop:** Please refer to the [Docker](#docker) section to complete this step.

4. **Launch the rover:** Run the following command inside the laptop's container to start all the necessary rover control nodes:
   ```bash
   ros2 launch rover_bringup rover.launch.py
   ```

5. **Connect the joystick:** Pair the joystick with your laptop and connect it.

6. **Verify the joystick connection:** Wait for the joystick LEDs to stop blinking and turn solid white, indicating a successful connection.

7. **Start the joystick nodes:** Run the following command inside the laptop's container to start the nodes responsible for handling joystick input:
    ```bash
    ros2 launch rover_teleop joy.launch.py
    ```

8.  **Control the rover:** You are now ready to drive the rover using the joystick.


### Turning Off the Rover

> **Warning:** Before cutting power, it is highly recommended to shut down the Jetson first by SSHing into it and running `sudo poweroff`. Wait about a minute, until the Jetson's fan and LED turn off, before cutting power. Cutting power without doing this can corrupt the Jetson's file system.

To power down the rover, perform **one** of the following actions:

* **Use the power switch:** Rotate the blue switch on the back of the rover counterclockwise.
* **Use the Emergency Stop:** Press one of the two red buttons on the top of the rover.

> **Note:** To restart the rover after the emergency button has been pressed, you must first release the button by rotating it clockwise.


### Current System Credentials and Network Info

| Property | Value |
| --- | --- |
| **Jetson Username** | `indomitus-rover` |
| **Jetson Computer Name** | `indomitus-rover-computer` |
| **Jetson Password** | `1` |
| **Wi-Fi Hotspot Name (SSID)** | `ERC_UCUSpaceRobotics_A` |
| **Wi-Fi Password** | `19283746` |
| **Jetson Static IP for Hotspot** | `10.42.0.1` |
| **Jetson Container ROS_DOMAIN_ID** | `90` |


### SSH Access to the Jetson

You can SSH into the Jetson to access its bash shell for debugging or configuration. You can use either the Jetson's hotspot or an Ethernet cable.

> **Note:** If you are prompted with a security fingerprint warning, type `yes` to continue. When asked for the password, enter `1`.

#### SSH via Hotspot

1. **Connect to the network:** Connect your computer to the Jetson's Wi-Fi hotspot. The credentials can be found in the [Current System Credentials and Network Info](#current-system-credentials-and-network-info) section.

2. **Initiate the connection:** Open your terminal and run the following command:
   ```bash
   ssh indomitus-rover@indomitus-rover-computer.local
   ```

#### SSH via Ethernet

1. **Prerequisites:** Ensure the prerequisites are satisfied. Refer to the **Laptop Setup** and **Jetson Setup** sections in [ssh.md](docs/networking/ssh.md).
   
2. Connect the Jetson to the laptop with an Ethernet cable, using the `ETH0` port on the Jetson.

3. Click the Network icon in your laptop's taskbar and select the **Jetson Tether** profile.

4. **Initiate the connection:** Open your terminal and run the following command:
   ```bash
   ssh indomitus-rover@indomitus-rover-computer.local
   ```

> **Note:** For further details on the network configuration, refer to [ssh.md](docs/networking/ssh.md).


---


## ROS_DOMAIN_ID

The **`ROS_DOMAIN_ID`** is a number that acts like a private radio channel for your ROS 2 network. It isolates different ROS 2 applications running on the same physical network, ensuring that only nodes sharing the exact same ID can discover and communicate with each other.

Because this ID is an environment variable that nodes read only at creation, it cannot be changed while a node is running. You must restart the node to apply a new ID.

* **Connecting to the Rover:** You do not always need to SSH into the Jetson. By connecting to the Jetson's hotspot and matching its `ROS_DOMAIN_ID` in your local container, your laptop and the rover share a network. You can monitor the system using standard tools (e.g., `ros2 topic list`) or even launch local nodes on your laptop that interact directly with the Jetson's nodes.
* **Avoiding Network Interference:** If multiple developers are working independently on the same Wi-Fi network using the default or identical IDs, their data will mix together and cause system issues. To prevent this, each developer must choose a **unique** `ROS_DOMAIN_ID`.

> **Note:** The current `ROS_DOMAIN_ID` assigned to the rover container can be found in the [Current System Credentials and Network Info](#current-system-credentials-and-network-info) section.

### Managing ROS_DOMAIN_ID

**To check your current ID:**
Run the following command in your terminal:
```bash
echo $ROS_DOMAIN_ID
```

**To change the ID and restart your container:**

1. Stop the currently running container:
   ```bash
   docker compose stop
   ```

2. Start the container with the new ID. You can pass the variable directly into the start command:
   ```bash
   ROS_DOMAIN_ID=90 docker compose up -d
   ```

*(Alternatively, you can permanently change the `ROS_DOMAIN_ID` value inside your `docker-compose.yml` file before running the standard `docker compose up -d` command).*


---


## Docker

### Getting the Docker Image

Before doing anything, you need to prepare your local environment:

1. Navigate to the root of the **indomitus-rover-core** repository.

2. Copy the example Docker Compose file:

   ```bash
   cp ./docker/docker-compose.dev.example.yaml ./docker-compose.yaml
   ```

> In most cases, the example Docker Compose file should work for you, but you may still need to modify it to match your machine.

Next, you need the Docker image. You can either pull a pre-built image or build it locally.

**Option A: Pull the image from GitHub**

*This may save significant time.*

For example, pull the image from `develop`:

```bash
docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:develop-dev
```

> **Pro Tip:** The images for the `develop` and `main` branches are built automatically, but you can also build an image for any branch. To do so, open the [GitHub Actions page](https://github.com/UCUSpaceRobotics/indomitus-rover-core/actions/workflows/publish_image.yaml). Press the gray button on the right, select the branch for which you want to build an image, and click **Run workflow**. When the image is built, you can pull it with the command `docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:<branch-name>-dev` (ensuring any slashes in your branch name are replaced with dashes, like `feature-shared-some-feature-dev`).

**Option B: Build the image locally**

```bash
docker compose build
```


### Start the Container and Build the Workspace

> **⚠️ Important:** All containers mount your local `src/` directory, which means your code will be used inside the container. Ensure that your `src/` directory contains the code you want to test. You may need to switch to the correct branch or pull the code.

1. Start the container in the background:

   ```bash
   docker compose up -d
   ```

   > **⚠️ Important:** By default, this starts the container using the image tag `local-dev` (defined in `docker-compose.yaml`). If you pulled the image from GitHub, it will have a different tag (refer to the [Image Tags](#image-tags) section). To start the container using a different image tag, you need to set the `IMAGE_TAG` environment variable. You can do this either in the `docker-compose.yaml` file or directly in the command:
   > ```bash
   > IMAGE_TAG=feature-shared-some-feature-dev docker compose up -d
   > ```
   > If you omit this, Docker will either build a new image (if there is no image tagged `local-dev`), or it will start the container using the existing `local-dev` image, which may have different dependencies installed.

2. Enter the running container:

   ```bash
   docker exec -it rover_dev /bin/bash
   ```

3. Build the ROS 2 workspace:

   ```bash
   cd /opt/ws && colcon build --symlink-install && source install/setup.bash
   ```

> **Note: Simulation Package Build Errors** At this stage, you might encounter build errors related to the `rover_viz` or `rover_sim` packages. This typically occurs because your current Docker container was built without the necessary visualization and simulation dependencies (controlled by the `INSTALL_SIM_TOOLS` variable in `docker-compose.yaml`).

Depending on your requirements, choose one of the following solutions:

**Option A: Skip the packages (If you do not need simulations)**
You can instruct `colcon` to ignore the failing packages and safely build the rest of the workspace:
```bash
colcon build --symlink-install --packages-ignore rover_viz rover_sim arm_viz arm_sim panel_sim && source install/setup.bash
```

**Option B: Fix the dependencies (If you need simulations)**
You must either manually install the missing dependencies inside your running Docker container or rebuild the entire Docker image with the simulation tools enabled.


### Image Tags

Understanding the tag naming convention will help you easily identify the correct Docker image for your hardware and environment.

The `Dockerfile` is split into two primary targets: **`dev`** and **`prod`**. The `prod` target strips out heavy dependencies used for simulations and visualizations (e.g., Gazebo, RViz) to keep the image lightweight for the rover.

Images built locally or by GitHub workflows will use the following tags:

* **`local-prod`**: Production image built locally.
* **`develop-dev`** (or `main-dev`): Development image built continuously by GitHub workflows.
  * *Architecture:* **AMD64**
  * *Use Case:* Can be used on standard Intel/AMD laptops for development and simulation. Cannot run on the Jetson.
* **`develop-prod`** (or `main-prod`): Production image built continuously by GitHub workflows.
  * *Architecture:* **ARM64**
  * *Use Case:* Designed strictly for deployment on the NVIDIA Jetson. Cannot run natively on standard Intel/AMD laptops.
* **`<branch-name>-dev`** and **`<branch-name>-prod`**: Images built by GitHub workflows when you manually trigger **Publish Production and Development Images** for your branch (slashes in the branch name are replaced with dashes).
* **`latest-prod`**: Production image built on merge to the main branch.


### Aliases

For convenience, the container includes built-in aliases for common ROS 2 commands. You can find the complete list in [aliases.md](./docs/software/aliases.md).


---


## Scripts

The repository contains utility scripts to simplify common workflow tasks. All scripts are located in the `scripts/` directory.


### Deployment Script

Use the dedicated deployment script to transfer your codebase and Docker environment to the Jetson on the rover.

To pull the latest image built by the CI pipeline on the `develop` branch, and transfer it along with the `src/` directory and `docker-compose.prod.yaml` file pulled from GitHub to the Jetson:

```bash
./scripts/deploy_to_rover.sh pull
```

The images for the `develop` and `main` branches are built automatically, but you can also use GitHub Actions to build an image for any other branch. To do so, open the [GitHub Actions page](https://github.com/UCUSpaceRobotics/indomitus-rover-core/actions/workflows/publish_image.yaml). Press the gray button on the right, select the branch for which you want to build an image, and click **Run workflow**. When the image is built, you can deploy it with the following command:

```bash
./scripts/deploy_to_rover.sh pull --tag <branch-name>-prod
```

Replace the branch name with your branch name, with slashes replaced by dashes.

**Building the image on GitHub and then deploying it with the script is the recommended deployment method.**

> **Note:** For further details on available deploy modes, refer to [deployment.md](docs/deployment.md). For full documentation, refer to [deploy_to_rover.md](docs/scripts/deploy_to_rover.md).


### Script to Enter Containers

This script automates the process of opening a bash terminal inside your Docker containers, whether they are running on your computer or on the physical robot.

Enter the **local** development container:
```bash
./scripts/enter_container.sh local
```

Enter the **remote** rover container:
```bash
./scripts/enter_container.sh rover
```

> **Note:** For further details, refer to [enter_container.md](docs/scripts/enter_container.md).


---


## Testing

Tests are automatically run by GitHub Actions when a PR is opened. To merge your PR, you need to ensure that all tests have passed. For details on how to view the results of tests run by GitHub Actions, or how to run tests locally, refer to [testing.md](./docs/software/testing.md).