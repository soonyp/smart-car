################################################################################
# MRS Version: 1.9.0
# �Զ����ɵ��ļ�����Ҫ�༭��
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/Seekfree_CH32V307VCT6_Opensource_Library/Seekfree_CH32V307VCT6_Opensource_Library/project/user/src/isr.c \
F:/Seekfree_CH32V307VCT6_Opensource_Library/Seekfree_CH32V307VCT6_Opensource_Library/project/user/src/main.c 

OBJS += \
./user_c/isr.o \
./user_c/main.o 

C_DEPS += \
./user_c/isr.d \
./user_c/main.d 


# Each subdirectory must supply rules for building sources it contributes
user_c/isr.o: F:/Seekfree_CH32V307VCT6_Opensource_Library/Seekfree_CH32V307VCT6_Opensource_Library/project/user/src/isr.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\Libraries\doc" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Core" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Ld" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Peripheral" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Startup" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\project\user\inc" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\zf_common" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\zf_device" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\project\code" -I"C:\Users\Universe\Desktop\Seekfree_CH32V307VCT6_Opensource_Library\project\code" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
user_c/main.o: F:/Seekfree_CH32V307VCT6_Opensource_Library/Seekfree_CH32V307VCT6_Opensource_Library/project/user/src/main.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\Libraries\doc" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Core" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Ld" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Peripheral" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\sdk\Startup" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\project\user\inc" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\zf_common" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\zf_device" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\project\code" -I"C:\Users\Universe\Desktop\Seekfree_CH32V307VCT6_Opensource_Library\project\code" -I"F:\Seekfree_CH32V307VCT6_Opensource_Library\Seekfree_CH32V307VCT6_Opensource_Library\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

