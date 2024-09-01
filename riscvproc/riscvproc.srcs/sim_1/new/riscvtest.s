# Test Program:
# If successful, it should write the value 0xAAAAA02E to address 100 (0x64) 
#       RISC-V Assembly     Description                 Address Machine Code
main:   addi x2, x0, 5      # x2 = 5                    0       00500113
        addi x3, x0, 12     # x3 = 12                   4       00C00193
        addi x7, x3, -9     # x7 = (12 - 9) = 3         8       FF718393
        or x4, x7, x2       # x4 = (3 OR 5) = 7         C       0023E233
        xor x5, x3, x4      # x5 = (12 XOR 7) = 11      10      0041C2B3
        add x5, x5, x4      # x5 = (11 + 7) = 18        14      004282B3
        beq x5, x7, end     # shouldn't be taken        18      02728863
        slt x4, x3, x4      # x4 = (12 < 7) = 0         1C      0041A233
        beq x4, x0, around  # should be taken           20      00020463
        addi x5, x0, 0      # shouldn't happen          24      00000293
around: slt x4, x7, x2      # x4 = (3 < 5) = 1          28      0023A233
        add x7, x4, x5      # x7 = (1 + 18) = 19        2C      005203B3
        sub x7, x7, x2      # x7 = (19 - 5) = 14        30      402383B3
        sw x7, 84(x3)       # [96] = 14                 34      0471AA23
        lw x2, 96(x0)       # x2 = [96] = 14            38      06002103
        add x9, x2, x5      # x9 = (14 + 18) = 32       3C      005104B3
        jal x3, end         # jump to end, x3 = 0x44    40      008001EF
        addi x2, x0, 1      # shouldn't happen          44      00100113
end:    add x2, x2, x9      # x2 = (14 + 32) = 46       48      00910133
        addi x4, x0, 1      # x4 = 1                    4C      00100213
        lui x5, 0x80000     # x5 = 0x80000000           50      800002b7
        slt x6, x5, x4      # x6 = 1                    54      0042a333
wrong:  beq x6, x0, wrong   # shouldn't be taken        58      00030063
        lui x9, 0xAAAAA     # x3 = 0xAAAAA000           5C      AAAAA4B7
        add x2, x2, x9      # x2 = 0xAAAAA02E           60      00910133
        sw x2, 0x20(x3)     # mem[100] = 0xAAAAA02E     64      0221a023
done:   beq x2, x2, done    # infinite loop             68      00210063
