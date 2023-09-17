# Isaac_ws guide

This part is for isaac_ws guide.

## Prequsite

Isaac_sim_2022.2.1

## Download files

you can download envs and robot files in this [link](https://www.dropbox.com/home/Auturbo/QNAI).

**Env files**

-   Hospital

**Robot files**

-   Carter
-   Unitree

you can download only one command.

```sh
sh setup_usd.sh
```

## How to start isaac sim python file?

```sh
./run.sh
```

# todo list

-   [x] Add isaac sim python file
-   [x] Spawn envs and robots
-   [ ] Make omnigraph for quadruped robot(Jinwon)
    -   [ ] Controller
    -   [ ] IMU, Lidar, Joint state to ros topic
-   [ ] Make omnigraph for Mobile robot (Minwoo)
    -   [ ] Controller
    -   [ ] IMU, Lidar, encoder to ros topic
