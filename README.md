#ra-l_2017

Under serious construction...

Please bear with me while I clean this up.

# Dependencies & Install

pip/pip3 install DIRECT emcee numpy scipy matplotlib cma sklearn

RoBO fork:

```
git clone -b corl_2017 https://github.com/rlober/RoBO
cd RoBO
pip/pip3 install .
```

Code:

```
git clone https://github.com/rlober/ra-l_2017.git
cd ra-l_2017
mkdir build
cd build
cmake ..
make install
```
```
git clone https://github.com/rlober/gttraj.git
cd gttraj
mkdir build
cd build
cmake ..
make install
```
```
git clone https://github.com/rlober/com-traj-gen.git
cd com-traj-gen
mkdir build
cd build
cmake ..
make install
```
Gazebo:

```
git clone -b corl_2017 https://github.com/robotology-playground/icub-gazebo-wholebody.git
```

WBIController:

```
cd $CODYCO_SUPERBUILD_ROOT
cd main/WBIToolboxControllers
git checkout corl_2017
cd ../../build
make
```


env variables:

```
export PYTHONPATH=$PYTHONPATH:[path]/ra-l_2017/optimization-scripts
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[path]/icub-gazebo-wholebody
```

## Generate python documentation:

```
cd [path_to_repo]/optimization-scripts/docs
sphinx-apidoc -f -o _source/ ../task_optim/
make html
```
