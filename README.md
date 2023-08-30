# Prevent errors when creating custom messages

Many ROS2 errors can occur when trying to setup custom messages. This repository was originally set up in order to communicate with the ROS2 maintainers, but I fortunately found out at least on how to set up a somehwat reliable environment to develop custom messages. Here is my incomplete checklist on how to counter those errors.

- **Check for mistakes:** The most obvious: you made an error yourself. Check your code very carefully. Especially dependencies omitted in CMakeLists.txt are a source of frustration. Follow tutorials and manual carefully
- **Python version:** Ros2 appears to be sensitive for the version of the python compiler, what worked for me is creating a new environment with python 3.10, e.g.: `conda create -n <environment name> python=3.10 numpy`. Tests with pyton 3.8, 3.9 and 3.11 resulted in errors.
- **Environment:** Switch to that environment: `conda activate <environment name>`
- **Install additional libraries:** `pip install empy lark catkin-pkg`
- **Clean workspace:** When errors occur clean up your builds: rm -rf for build, install and log or better: `colcon clean workspace -y`. I once cleaned up src as well, so colcon clean is the safer option
- **New terminal:** In the case of errors also start from a new terminal, the working terminal has too many variables set (by `source install/setup.bash`) which are in some cases confusing or wrong. Don't forget to activate your environment
- **Anaconda is ok:** There was a remark about anaconda not trusted, no evidence was provided. I found anaconda very helpful in quickly setting up and testing environments for specific versions of python. Pip and anaconda interact without problems. Of course you may prefer other tools. Use what works best for you but if you use anaconda there is no need to switch
- **Try not to pip install as root:** Rosdep installs python dependencies as root  by default. In general this is not what you want and pip warns against it because it might break installations. In any case it prevents working with environments which I personally found very beneficial. To prevent rosdep installing python libraries as root use the rosdep option: `--as-root pip:false`

## My personal ros2 manual and cheat sheet

What makes a ROS package?


-    package.xml file containing meta information about the package
-    resource/<package_name> marker file for the package
-   setup.cfg is required when a package has executables, so ros2 run can find them
-   setup.py containing instructions for how to install the package
-    <package_name> - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py

The simplest ROS package contains the following files:

```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```

Best practice is to create new packages in the workspace/src directory. 

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

### 1. Create a package

```
cd workspace_folder/src
ros2 pkg create --build-type ament_python <package_name>
```

Use the optional argument `--node-name` to creates a 
simple Hello World type executable in the package.

```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

## 2 Check dependencies

Dependencies are listed in `package.xml` and checked by rosdep from the workspace as follows:

As root:

```
rosdep install --from-paths src -y --ignore-src
```

As user:

```
rosdep install --as-root pip:false --from-paths src -y --ignore-src
```

The --as-root option allows for installing packages not as root which can be very useful for pip installations. 

Dependencies for python are defined in `package.xml` by the \<test_depend> tag. 

<test_depend>python3-pytest</test_depend>


## 3. Build a package

```
colcon build --packages-select my_package
```

## 4. Source the setup file

Add the workspace to the path.

```
source install/local_setup.bash
```

## 5. Run the package

To run the executable you created using the --node-name argument during package creation, enter the command:

```
ros2 run my_package my_node
```

## 6. Examine package contents

Inside ros2_ws/src/my_package, you will see the files and folders that ros2 pkg create automatically generated: `my_package  package.xml  resource  setup.cfg  setup.py  test`.

## 7. Customize the package

### package.xml

```
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <test_depend>ament_copyright</test_depend>
 <test_depend>ament_flake8</test_depend>
 <test_depend>ament_pep257</test_depend>
 <test_depend>python3-pytest</test_depend>

 <export>
   <build_type>ament_python</build_type>
 </export>
</package>
```

Change at least the following lines:

- `maintainer` - maintainer email and name
- `description` - give a good description of the package
- `license` - apply the coorect license
- `_depend` - dependencies of the package

### setup.py

Change 

- `maintainer` - maintainer name
- `maintainer_email` - maintainer email
- `description` - give a good description of the package
- `license` - apply the coorect license
- `entry_points` - supply the `console_scripts` with which to start yout package

```
    entry_points={
        'console_scripts': [
                'my_node = my_py_pkg.my_node:main'
        ],
    },
```

8. Issues with creating custom messages/topics

Best is to create a stand-alone project for C++. Adjust CMakeLists.txt and package.xml accordingly and build. Typical build errors have been solved [here](https://stackoverflow.com/questions/72752937/ros2-importerror-cannot-import-name-generate-py-from-rosidl-generator-py). In short:

install two packages in your environment:

- pip install empy
- pip install lark

If that is insuffient comment out the next line in CMakeLists.txt, so:

    # find_package(rosidl_default_generators REQUIRED)