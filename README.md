# my-actuator-cntl

This repo controls four x6-40 motors with CAN messages triggerd from a rostopic via a callback function. 

A secondary script monitors the CAN bus for motor replies containing motor data that is published to their corresponding rostopics.
