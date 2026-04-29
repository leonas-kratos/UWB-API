# Interface Control Document (ICD) for Parsing Tag-Distance Messages
This Document explains how to parse received messages from the tag using UART.
	
	
## Message Format
Each message contains pairs of TagID and distance, separated by "|". The format for each pair is `0x[6-byte TagID]: [distance in millimeters]`.


## Parsing Rules
1. Split the message into pairs using "|".
2. Extract TagID and distance for each pair by splitting with ": ".
3. Validate TagID format (6-byte hexadecimal) and distance (non-negative integer).
4. Convert TagID to numerical format and distance to integer.

## Data Representation Table

| Field     | Description                                 | Format              |
|-----------|---------------------------------------------|---------------------|
| TagID     | Identifier for the tag                      | 6-byte hexadecimal  | 
| Distance  | Measured distance to the tag in millimeters | Integer             | 

### Example of a message:
```
0x1234: 382 | 0x6789: 1038 |
```
0x1234: first tag id 
382: the distance in millimeters to the first tag
0x6789: second tag id 
1038: the distance in millimeters to the second tag
