# Load a bunch of values into memory locations 120 to 132
addi x1, x0, 17
addi x3, x0, 19
addi x5, x0, 21

sw x1, 120(x0)
sw x3, 124(x0)
sw x5, 128(x0)

lw x2, 120(x0)
lw x4, 124(x0)
lw x6, 128(x0)

addi x1, x0, 10
sw x1, 132(x0)
lw x2, 132(x0)

addi x1, x0, 11
sw x1, 132(x0)
lw x2, 132(x0)

addi x1, x0, 12
sw x1, 132(x0)
lw x2, 132(x0)

addi x1, x0, 1
addi x3, x0, 2
addi x5, x0, 3

#addi x1, x1, 17
#sw x1, 124(x0)
#addi x1, x1, 17
#sw x1, 128(x0)
#addi x1, x1, 17
#sw x1, 132(x0)
# Read them back out into registers for easy debugging.
#lw x2, 120(x0)
#lw x3, 124(x0)
#lw x4, 128(x0)