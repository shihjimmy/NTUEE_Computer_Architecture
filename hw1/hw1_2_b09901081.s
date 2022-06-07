.globl __start

.rodata
    msg0: .string "This is HW1_2: \n"
    msg1: .string "Plaintext: "
    msg2: .string "Ciphertext: "
.text

################################################################################
# print_char function
# Usage: 
#     1. Store the beginning address in x20
#     2. Use "j print_char"
#     The function will print the string stored from x20 
#     When finish, the whole program will return value 0
print_char:
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    add a1, x0, x20
    ecall
# Ends the program with status code 0
    addi a0, x0, 10
    ecall
################################################################################
################################################################################
__start:
# Prints msg
    addi a0, x0, 4
    la a1, msg0
    ecall
     
    la a1, msg1
    ecall
    
    addi a0, x0, 8
    
    li a1, 0x10130
    addi a2, x0, 2047
    ecall
# Load address of the input string into a0
    add a0, x0, a1
################################################################################   
################################################################################ 
# Write your main function here. 
# a0 stores the beginning Plaintext
# Do store 66048(0x10200) into x20 

addi sp, sp, -8    # move stack pointer for the temporary variable 
sw   x23, 0(sp)
addi x23, x0, 0

addi x24, x0, 9    # deal with the comma problem (decrease from 9 to 0)
addi x25, x0, 44   # store the comma ASCII code 
addi x27, x0, 122  # store z ASCII code 
addi x21, x0 ,48   # store 0 ASCII code
addi x22, x0, 10   # store line feed return ASCII code

L1: 
    add x5, x23, a0   # get the plain text's address
    addi x7, a0, 208  # turn x7 into 10120
    add x7, x7, x23   # increment x7 to the right position
    lbu x6, 0(x5)     # x6 = char in plain text
    addi x19, x6, 0   # put plain text in x5 but answer in x7
    beq  x19, x22, L2     

ciphertext:
    beq  x19, x25, comma     # if char == ","
    blt  x19, x27, alphabet  # if char is a alphabet

comma:
    add x19, x24, x21       
    sb  x19, 0(x7)          # store the number into add:0(x7)
    addi x24, x24, -1
    addi x23, x23, 1        # a char == a byte 
    jal L1  
    
alphabet:
    addi x19, x19, 13
    addi x27, x27, 1
    bgeu x19, x27, over     # cope with the alphabet over the range of a to z
    addi x27, x27, -1
    addi x19, x19, -32 
    sb   x19, 0(x7)
    addi x23, x23, 1
    jal L1

over:
    addi x27, x27, -1
    sub  x19, x19, x27
    addi  x19, x19, 64       # A + overflow
    sb   x19, 0(x7)
    addi x23, x23, 1
    jal L1

L2:
    lw  x23, 0(sp)
    addi sp, sp, 8
    add x20, x0, a0
    addi x20, x20, 208        # 256 - 48 = 208 
    j print_char              # store 10200 to the x20(original is 10130)

################################################################################    
