Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.* 

1. What is the average current per period?
   Answer:22.21 uA
   <br>Screenshot:  
   ![Avg_current_per_period](https://github.com/CU-ECEN-5823/ecen5823-assignment4-AkankshaTripa/blob/master/questions/Assignment4_Screenshots/Avg_current_per_period.png)  

2. What is the average current when the Si7021 is Powered Off?
   Answer: 2.72 uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](https://github.com/CU-ECEN-5823/ecen5823-assignment4-AkankshaTripa/blob/master/questions/Assignment4_Screenshots/Avg_current_LPM_Off.png)  

3. What is the average current when the Si7021 is Powered On?
   Answer:554.78 uA
   <br>Screenshot:  
   ![Avg_current_LPM_On](https://github.com/CU-ECEN-5823/ecen5823-assignment4-AkankshaTripa/blob/master/questions/Assignment4_Screenshots/Avg_current_LPM_On.png)  

4. How long is the Si7021 Powered On for 1 temperature reading?
   Answer:105 mS
   <br>Screenshot:  
   ![duration_lpm_on](https://github.com/CU-ECEN-5823/ecen5823-assignment4-AkankshaTripa/blob/master/questions/Assignment4_Screenshots/duration_lpm_on.png)  

5. Compute what the total operating time of your design for assignment 4 would be in hours, assuming a 1000mAh battery power supply?
   Answer: 
                  From the readings the average current per period is = 22.21 uA
		

                  Current for 1 hour=(22.21*3600*0.001)/3000 
					      =26.66mA

                  Total time= 1000mAh/(26.66mA)= 37.50 hrs
			
			So, total operating time for design is 37.50 hrs
   
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer: 
			The power consumption performance has decreased significantly from the previous assignment due to the use of interrupt(in this assognment) 			instead of polling(previous assignment). Also, to save power the system runs in EM3 mode and for I2c transfer it goes to  EM1 mode. The 				value of average current per period was 144 uA in last assignment and here its 22.21uA which is almost 6.48 times reduced than of the last 			value. Also, the average current when the Si7021 is Powered On was 4.54 mA and here it is 554.78 uA so again the current is reduced by a 				factor of almost 8.18. When temperature sensor is off the average current in this assignment is 2.72 uA while in the previous assignment 				it was 3.57uA so there is a slight change of almost 0.85uA here as well 
   
7. Describe how you tested your code for EM1 during I2C transfers.
   
Answer:   
            I have tested the code in debug mode as well as in energy profiler and the result verifies that the system is sleeping in EM1 mode during i2c 			transfer. For testing in debug mode I added breakpoint in i2c IRQ handler, once the program is executed and we hit the breakpoint then when 			we debug it stepwise it directs us to entered mode as "sleepEM1" which proves that the system sleeps in EM1 mode before interrupt occurs. 			Further, to verify that the system is entering EM3 sleep mode during sleep delays and while waiting for the next underflow interrupt, a 				breakpoint was placed inside the LETIMER0 IRQ handler when the scheduler event is set. After stepping through the code, it was found that the 			modeEntered variable was set to sleepEM3, indicating that the system was indeed in EM3 sleep mode during those periods of idle time.  
		Also, this change can be observed through energy profiler as well, it is observed that during I2C transfers, there was a spike in the amount of 		time the LETIMER0 was waiting, and this time was programmed for EM1 sleep mode. However, in between these spikes, there was a reduction in 		      current consumption, indicating that the system was indeed in EM1 mode during I2C transfer. 
