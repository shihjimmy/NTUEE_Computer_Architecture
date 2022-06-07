.globl __start

.rodata
    msg0: .string "This is HW1-1: \nT(n) = 2T(3n/4) + 0.875n - 137, n >= 10\n"
    msg1: .string "T(n) = 2T(n-1), 1 <= n < 10\nT(0) = 7\n"
    msg2: .string "Enter a number: "
    msg3: .string "The result is: "
.text

__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall
  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall
  # Reads an int
    addi a0, x0, 5
    ecall

################################################################################ 
  # Write your main function here. 
  # The input n is in a0. 
  # You should store the result T(n) into t0.
  # Round down the result of division.
  
  # HW1_1 
  # T(n) = 2T(3n/4) + 0.875n - 137, n >= 10
  # T(n) = 2T(n-1), 1 <= n < 10
  # T(0) = 7

jal x1, FUNCTION
jal x0, result

FUNCTION:
    addi sp, sp, -8     # reserve space for n and return address on the stack
    sw  ra, 0(sp)       # save the return address
    beq a0, x0, case3
    sw  a0, 4(sp)       # save n
    
    addi x19, x0, 10
    bge  a0, x19, case1
    addi x6, x0, 1
    bge  a0, x6, case2  # n-1 >= 0 or not
    jal  x0, EXIT
    
case1:
    add  x18, a0, x0
    srli x18, a0, 3    #  0.125n
    sub  x18, a0, x18  #  n-0.125n
    addi x18, x18, -137

    add  x17, a0, x0 
    srli a0, a0, 2
    sub  a0, x17, a0    
    jal  FUNCTION
    slli t0, t0, 1
    add  t0, t0, x18
    jal  x0, EXIT

case2:
    addi a0, a0, -1
    jal  FUNCTION
    slli t0, t0, 1
    jal  x0, EXIT

case3:
    addi t0, x0, 7      # load 7 for the answer

EXIT:
    lw ra, 0(sp)        # get the return address back
    addi sp, sp, 8      # restore the stack
    jalr x0, ra, 0
    
################################################################################

result:
  # Prints msg2
    addi a0,x0,4
    la a1, msg3
    ecall
  # Prints the result in t0
    addi a0,x0,1
    add a1,x0,t0
    ecall
  # Ends the program with status code 0
    addi a0,x0,10
    ecall    