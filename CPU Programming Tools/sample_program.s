//This is a sample assembly program for CO224 Lab 5
loadi 4 0x05    // r4 = 5
loadi 2 0x09    // r2 = 9
add 6 4 2       // r6 = r4 + r2
mov 0 6			// r0 = r6
loadi 1 0x01	// r1 = 1
add 2 2 1		// r2 = r2 + r1 (r2++)
//j 0xFB		// jump to line 4