<!-- markdownlint-disable MD013 -->
# Initial Plan

## We are modeling radar

You are the main character in a spy movie and have tracked down a criminal maple syrup syndicate to Mount Thor, the largest cliffside in Canada.
Your helicopter was able to drop you off at the top of the cliff with some radar equipment.
The criminal maple syrup kingping along with 4 other security cars (5 cars total) are passing by this cliff.
Only one of the 5 cars is carying large amounts of high quality saffron maple syrup.
You are tasked with identifying which car is the one containing premium saffron maple syrup and which arent.
The cars are currently passing by under you, all at around the same velocity.
They look identical and there is a fork in the road that leads down 5 paths.
The cars have split up and are all going in different dirictions.
You want to save the day and skydive down to the correct car carrying the syrup, however, you can't tell which one is which.
However, you do know that they are traveling down a gravel road. The car containing the maple syrup is much heavier than the rest, so they frequencies that it vibrates are at are different.

As a spy who also has an EE degree, you know this.
In-fact, you planned for this.
You brought your handy handy 64 Element Uniform Linear Array Radar 77 GHz radar kit and are tasked to develop a DSP algorithm that can identify which of the 5 cars is carrying the maple syrup.
After you do this, you next want to predict where the car will be in the next 15 seconds (the time it will take you to skydive) so you can properly aim yourself and hijack the carivan.

## System Description

Tracking: 30s,  ALL Cars are moving in the same line

Split: 15s, Each of the 5 cars pick a direction and vary their velocity

Predict: 15s, After the last bit of information, you need to predict where you are going to jump

1. 77 GHz Pulse Sent out from an 8x8 ULA Radar Array
2. 77 GHz Pulse Received from system
3. IQ Modulated Signal  (x MHz) of block size (___)
4. ?? You decide

## Input

- IQ Modulated Pulse 8x8 RX Signal

## Output

- Time and location on where to jump after first 45s.
