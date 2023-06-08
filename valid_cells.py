with open('binary_image.txt', 'r') as infile, open('output.txt', 'w') as outfile:
    for i, line in enumerate(infile):
        for j, val in enumerate(line.split()):
            if val == '1':
                outfile.write('({}, {})\n'.format(i, j))

