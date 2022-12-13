addi x1, x0, 15
jal x6, label_a
addi x1, x0, 16
label_a: addi x1, x0, 17
jalr sp, x6, 0

addi x10, x0, 15
addi x11, x0, 15
beq x10, x11, label_b
addi x12, x0, 16
label_b: addi x13, x0, 17