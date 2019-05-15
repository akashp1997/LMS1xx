To use this module do the following:
- Create an executable C++ shared library
    1. Create a folder called `build` and cd to it
    2. Make sure Cmake is installed on the system
    3. Write the command `cmake ../` and let it do its job
    4. Write the command `make` in this directory
    5. A new `.so` will be made in this folder, copy it to the folder where lms1xx_mq.py exists
- Use it with `lms1xx_mq.py`
    1. Copy the `.so` file to the same folder as `lms1xx_mq.py`
    2. Run the python file
