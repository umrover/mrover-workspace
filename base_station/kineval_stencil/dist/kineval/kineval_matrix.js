// REDO THIS FILE


//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z


function matrix_multiply(m1, m2){
    return math.multiply(math.matrix(m1), math.matrix(m2))
}

function matrix_transpose(m1) {
    return math.transpose(math.matrix(m1))
}


function matrix_pseudoinverse(m1) {

}

//   matrix_invert_affine

function vector_normalize(v1){
    return math.norm(math.matrix(v1))
}

function vector_cross(v1,v2){
    return math.cross(math.matrix(v1), math.matrix(v2))
}


function generate_identity(k){
    var i = []
    for (var i = 0; i < k; ++i)
        i.push(1)
    return math.diag(i)
}

function generate_translation_matrix(x,y,z){
    var i = math.diag([1,1,1,1])
    i[0][3] = x 
    i[1][3] = y 
    i[2][3] = z 
    return i
}

function generate_rotation_matrix_X(angle){
    var rot = math.diag([1,1,1,1])
    rot[1][1] = math.cos(angle)
    rot[1][2] = -math.sin(angle)
    rot[2][1] = math.sin(angle)
    rot[2][2] = math.cos(angle)
    return rot
}

function generate_rotation_matrix_Y(angle){
    var rot = math.diag([1,1,1,1])
    rot[0][0] = math.cos(angle)
    rot[2][0] = -math.sin(angle)
    rot[0][2] = math.sin(angle)
    rot[2][2] = math.cos(angle)
    return rot 
}

function generate_rotation_matrix_Z(angle){
    var rot = math.diag([1,1,1,1])
    rot[0][0] = math.cos(angle)
    rot[0][1] = -math.sin(angle)
    rot[1][0] = math.sin(angle)
    rot[1][1] = math.cos(angle)
    return rot
}


///EDIT by myself:
function matrix_vec_multiply(m1,v1){

}

function vector_dot(v1,v2){

}

function vec_norm2(v1) {

}

function vec_minus(v1,v2) {

}

function vec_norm2_diff(v1,v2) {
}

function generate_rotation_matrix (r,p,y){

}

///EDIT by myself for advanced points of LU decomposition:
function matrix_inverse(m1){

}

function linear_solved(m1,b1){

}

function LU_dec(m1){

    return{l: mL, u: mU};
}

