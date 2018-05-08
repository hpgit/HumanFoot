def to_int(x):
    return int(x)-1


def import_obj(filename, scale=1.):
    f = open(filename, 'r')
    lines = f.read().split('\n')
    f.close()

    ver_coord = []
    nor_coord = []
    tex_coord = []

    tri_face_idx = []
    quad_face_idx = []

    for line_num in range(len(lines)):
        line = lines[line_num]

        if len(line) == 0 or line[0] == '#':
            continue
        elif line[0] == 'v' and line[1] == ' ':
            string = line.split()
            # ver_coord.append(tuple(map(float, string[1:4])))
            for i in range(1, 4):
                ver_coord.append(scale * float(string[i]))

        elif line[0] == 'v' and line[1] == 'n':
            string = line.split()
            # nor_coord.append(tuple(map(float, string[1:4])))
            for i in range(1, 4):
                nor_coord.append(float(string[i]))

        elif line[0] == 'v' and line[1] == 't':
            string = line.split()
            # tex_coord.append(tuple(map(float, string[1:3])))
            for i in range(1, 3):
                tex_coord.append(float(string[i]))

        elif line[0] == 'f' and line[1] == ' ':
            string = line.split()
            if len(string) == 4:
                tri_face_elem = list()
                for i in range(1, 4):
                    # tri_face_elem.append(tuple(map(to_int, string[i].split('/'))))
                    tri_face_idx.append(to_int(string[i].split('/')[0]))
                # tri_face_idx.append(tuple(tri_face_elem))

            elif len(string) == 5:
                quad_face_elem = list()
                for i in range(1, 5):
                    quad_face_idx.append(to_int(string[i].split('/')[0]))
                # quad_face_idx.append(tuple(quad_face_elem))

    return ver_coord, nor_coord, tex_coord, tri_face_idx, quad_face_idx


if __name__ == '__main__':
    ver, nor, tex, face_tri, face_quad = import_obj('femur_R.obj')
    print(face_quad)
