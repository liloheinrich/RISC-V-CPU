addi x1, x0, 15
addi x2, x0, 15
beq x1, x2, label_a
addi x3, x0, 16
label_a: addi x4, x0, 17



#addi x1, x0, 15
#jal x6, label_a
#addi x1, x0, 16
#label_a: addi x1, x0, 17
#jalr sp, x6, 0