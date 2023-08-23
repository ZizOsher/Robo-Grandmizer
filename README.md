# Robo-Grandmizer

Dependencies:
make sure you have nlohmann-json installed. You can easily do so like this:
    sudo apt update
    sudo apt install nlohmann-json3-dev

Build and Compile:
From the project's base directory run:
mkdir build && cd build
cmake ..
make

Running Compiled Program:
1. First of all, open cs_floor_nav.cfg and make sure that the path for stageplugin.so (Line 5) corresponds to where it's installed on your computer. If not, replace the given path with thelocation of stageplugin.so on your computer.
2. Now, you are going to need two terminals open - one to run player/stage and the other to run the compiled client.
3. If your current working directory is not the base directory fot the project change your current directory to it. Now in the first terminal run:

    player cs_floor_nav.cfg

4. One Player/Stage is up and running, Run the following in the second terminal:
    cd build
    ./client

5. Now simply follow the command line prompts.
