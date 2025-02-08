## Simple SSH setup in VScode with Mac OS
Following tutorial: [link](https://code.visualstudio.com/docs/remote/ssh-tutorial#_remote-ssh)

1. Install Remote - SSH extension in VScode extensions <br>
   A blue status bar will appear at the bottom left corner
2. Create the remote **v**irtual **m**achine (VM)<br>
   ***Important***: keep the username during creating the VM<br>
   ***Important***: add SSH public key to the VM<br>
     - keep the passphrase when generating the SSH key, which is required for SSH connection<br>
3. Specify the public IP address of VM
4. Establish the SSH connection
    - **Option 1**: test the connection in command line
    ```bash
    ssh username@hostpublichIP
    # example: ssh xinlei@172.190.229.141
    ```
    - **Option 2**: establish the SSH connection in VScode<br>
        1. Click the remote status bar at the bottom left corner to bring up the common remote - SSH commands
        2. Select the `connect to HOST`
        3. Follow the instruction, input the `username@hostpublichIP`, and passphrase to establish the SSH connection
        4. The remote status bar at the bottom left corner will indicate the host public IP if connection succeed
5. Open the folder of VM in local VScode
   - input `>file:Open Folder` in VScode command palette

## Port forwarding
Following tutorial: [link](https://code.visualstudio.com/docs/remote/ssh#_temporarily-forwarding-a-port)

***Goal***: To be able to access a port on the remote machine that may not be publicly exposed, you need to establish a connection or a tunnel between a port on your local machine and the server.

1. Select `Forward a Port` from the Command Palette or select the `Forward a Port` button in the Ports view. You can see the Ports view in the bottom panel. 
2. Input the port number that needed to be forwarded
3. A local address will appear right to the specified port number
