# **UncertanTIC**

Folder containing the code to transmit the data by the sensors to a IoT Hub and save he historical data

## **Directory organization**

- *BS_IoT.py* : file which receives the data form the MR and sends it to a IoT Hub. It also saves the data into the folder /Logs

- /**Logs** : folder where the historical data from the sensors is saved. This folder is cleared with every *BS_IoT.py* execution.
