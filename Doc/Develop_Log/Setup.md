# Webots setup for Linux

## 1. Download 

> ref:[Download Webots on linux](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Use `snap` to download Webots. 

By using `gedit ~/.bashrc` to open the text. Then add `export WEBOTS_HOME=/snap/webots/current/usr/share/webots:$WEBOTS_HOME # Defines the path to Webots home.` to the end of the text.

In Vscode, add `"${WEBOTS_HOME}/include/controller/cpp/**"` to `"includePath"` in the file called `c_cpp_properties.json`.

## 2. Connect Github with Linux

1. Generate an SSH code on your Linux end.
2. Add your Linux PC to your GitHub account.
3. Setup vscode on Linux.
4. Git clone with ssh.
I AM DONE!

# Set up the main function in Cpp

> Thanks to PriyankaPrakashChand's [GitHub Repository](https://github.com/PriyankaPrakashChand/Micromouse_E-Puck). The Cpp version of the main function for this Webots project is beautifully built. 


