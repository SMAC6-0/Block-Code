 Written by Isa Kaplan (email gershonisa@gmail.com with any questions)

Code Editing:
We (SMAC 5.0) used Thonny to edit code, would recommend. 

Setting up new Pi Pico:
When programming a fresh Pi Pico, first the libraries must be installed. They are contained in the folder
"BlockPackage" the three folders within this one must be uploaded to the Pico. This can be done by right
clicking on the folders in the file navigation bar on the left side of Thonny and selecting "upload to."
In order to have the Pico run code upon being powered on, the desired code must be saved to the Pico as "main.py."
This can be done by clicking file->save as and then selecting the Pi Pico in the box that pops up.

IR code source:
tx code and rx code were adapted from two seperate sources
original rx code was found here: https://github.com/sunfounder/euler-kit/tree/main/micropython
original tx code was found here: https://github.com/peterhinch/micropython_ir