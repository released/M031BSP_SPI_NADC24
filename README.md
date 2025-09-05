# M031BSP_SPI_NADC24
M031BSP_SPI_NADC24

update @ 2025/09/05

1. init ADC0_CH2 : PB2 , monitor target ADC channel for compare

2. init SPI master SPI0 , to drive NADC24 
	
	- PA0(MOSI)
	
	- PA1(MISO)
	
	- PA2(CLK)
	
	- PA3(SS)
	
	- PA4(nDRDY) , initial by GPIO IRQ

3. reference sample code

https://forum.nuvoton.com/zh/mcu-nudeveloper-ecosystem/bsp-example-code/12760

https://forum.nuvoton.com/zh/mcu-nudeveloper-ecosystem/bsp-example-code/12753

4. control flow : 

	- set channel (refer to nadc_set_channel)
	
	- set cmd : start convert (refer to nadc_set_channel)
	
	- wait for nDRDY low (LOW ACTIVE) , MUST wavie the first few convert data (refer to nadc_single_ended_process , nadc_differential_process)
	
	- set cmd :　read ADC result (refer to nadc_readADCdata)
	
	- convert the raw data into voltage
	
	- set NEXT channel (refer to nadc_set_channel)
	
	- ...
	
5. check define : TEST_SINGLE_ENDED , TEST_DIFFERENTIAL

6. below is log message

power on log 
![image](https://github.com/released/M031BSP_SPI_NADC24/blob/main/log.jpg)

when measure single ended channel
![image](https://github.com/released/M031BSP_SPI_NADC24/blob/main/log_single_ended.jpg)

when measure differential channel
![image](https://github.com/released/M031BSP_SPI_NADC24/blob/main/log_differential.jpg)

7. wire connection
![image](https://github.com/released/M031BSP_SPI_NADC24/blob/main/Single_End_Connection.png)

