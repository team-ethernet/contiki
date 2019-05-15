At line 788 we subscribe to the topic "noisesensor/*current node ID*/sensors"

The subscribe function can be found at line 682

When a message is received it is stored in the variable pointer _chunk_ on line 435. 
This variable contains the message we will decode with our API. 
