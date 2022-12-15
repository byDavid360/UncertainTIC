# **UncertainTIC**

## **/GNURAdio Files**

- *DistEst_Left.grc* : GNU Radio Companion flowgraph for estimating distance to transmitting antenna using simple free path loss model. This one is meant to use left MR antenna patch for the calculation.

- *DistEst_Left.py* : Python file generated from GNU Radio Companion to run the *DistEst_Left.grc* flowgraph.

- *DistEst_Right.grc* : GNU Radio Companion flowgraph for estimating distance to transmitting antenna using simple free path loss model. This one is meant to use right MR antenna patch for the calculation.

- *DistEst_Right.py* : Python file generated from GNU Radio Companion to run the *DistEst_Right.grc* flowgraph.

- *DoA&DistEst.grc* : GNU Radio Companion flowgraph for the DoA (Direction of Arrival) and the following distance estimation. Using a arbitrary antenna patch for the distance estimation.

- *DoADistEst.py* : Python file generated from GNU Radio Companion to run the *DoA&DistEst.grc* flowgraph.

- Rest of .py files are the scripts that run each Emmbedded Python Block that goes inside respective grc flowgraph.
