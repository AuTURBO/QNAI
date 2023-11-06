# QNAI

This project is a Framework for quadruped robot in isaac sim.

## Settings

You need to clone this project inside the folder of your Isaac Sim project.
e.g.:

```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1
```

you need to download asset from [google drive](https://drive.google.com/drive/folders/1Yxivv1F7GC0nwLJlQh1xYkyHt_tSz_pp?usp=sharing) and put it in the Asset folder.

1. hospital.usd

```bash
# locate the directory below
isaac_ws/Assets/Envs
```

2. go1.usd

```bash
# locate the directory below
isaac_ws/Assets/Robots
```

3. anymal_c.usd + materials

```bash
# locate the directory below
isaac_ws/Assets/Robots
```

Now you can easily launch the python file in the isaac_ws folder using VScode.

if you want to know more details, show the [isaac sim documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/install_python.html).

## Launch python files

![image](Asset/1.png)
you can launch python file using left upper button "Python: Current File"

## Test pylint

if you want to check your code using pylint, you can use the command below:

```sh
sh test_pylint.sh
```
