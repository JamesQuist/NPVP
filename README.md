# NPVP
Negative Pressure Ventilator Project

The Negative Pressure Ventilator Project aims to create a modern low-cost negative pressure ventilator.

Most ventilators on the market today are positive pressure ventilators; they push air into the lungs via a mask or tracheostomy in order to ventilate the individual. In contrast, negative pressure ventilators pull away the air that surrounds the torso, which causes the chest and lungs to expand and air to flow into the airway in a more natural manner. While negative pressure ventilators are often associated with the antiquated iron lung used during the polio era, advancements in 3D printing, scanning, and textiles allow for a more lightweight, customizable, and mobile design to be created. This project aims to make a proof of concept for a battery powered ventilator as well as a lightweight cuirass for a person to wear. 

As this is just an open source project and not a device that has been rigorously tested or regulated, it’s important to state it should not be used on any person in serious need and should be restricted to only artificial lung setups and labs. 

I believe it’s always better to have more options out there so that a person in need isn’t forced into a quality of life they don’t want. I hope this project can help build a path towards a device that can someday get the resources and funding it needs to enter the medical market.


Hardware:

1. 24V Micronel Radual Blower:

https://www.micronel.com/en/productfinder/radial-blower-u65hn-024ks-6/

The current ventilator design uses a 24V U65HN-024KS-6 blower. Since 12V batteries are very common, a switch to a 12V blower might be best for the mobile version.


2. The Micronel Evaluation Motor Driver Box MDB-48/10:

https://www.micronel.com/en/products/electronics-accessories/

This allows for the computer to interface with the blower via the free micronel proprietary software. This is a good first step as it allows for testing blower speeds and power usage. Ultimately, the goal is to move to open source software on an embedded board. 


3. Stm32 f767 nucleo-144 board

https://www.st.com/en/evaluation-tools/nucleo-f767zi.html

This is the board used for prototyping the embedded software. Depending on the needs of the ventilator design, the final design could be paired down to a m4 chipset. 


Software:

1. Micronel Evaluation Motor Driver Box Software (PC only)

https://www.micronel.com/wp-content/uploads/2024/01/MDB-48-10-Firmware-and-software.zip
