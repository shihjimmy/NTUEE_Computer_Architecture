.data
    n: .word 11
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    addi a0, x10, 0
    addi sp, sp, -8
    sw x1, 0(sp)

    addi a1, x0, 10
    addi a2, x0, 1
    addi a3, x0, 3
    addi a4, x0, 7
    addi a6, x0, 4
    addi a7, x0, 8
    jal x1, func

    addi x10, t1, 0
    lw x1, 0(sp)
    addi sp, sp, 8
    jalr x0, 0(x1)
func:
    addi sp, sp, -16
    sw a0, 8(sp)
    sw x1, 0(sp)
    bge a0, a1, large
    bge a0, a2, middle
    addi t1, x0, 7
    addi sp, sp, 16
    jalr x0, 0(x1)
large:
    mul a0, a0, a3
    srli a0, a0, 2
    jal x1, func
    lw x1, sp, 0
    lw a0, sp, 8
    addi sp, sp, 16
    mul a5, a0, a4
    srli a5, a5, 3
    slli t1, t1, 1
    add t1, t1, a5
    addi t1, t1, -137
    jalr x0, 0(x1)
middle:
    addi a0, a0, -1
    jal x1, func
    slli t1, t1, 1
    lw x1, 0(sp)
    lw a0, 8(sp)
    addi sp, sp, 16
    jalr x0, 0(x1)

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall