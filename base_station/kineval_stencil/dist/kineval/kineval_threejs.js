//////////////////////////////////////////////////
/////     threejs SUPPORT ROUTINES
/////       data structure conversion
/////       apply transforms to objects
/////
/////     ASSUME: threejs included in html
//////////////////////////////////////////////////


function matrix_threejs_to_2Darray(tmat) {
// conversion of threejs 4x4 matrix to 2D array

    var te = tmat.elements;
    //var amat = [[te[0],te[1],te[2],te[3]], [te[4],te[5],te[6],te[7]], [te[8],te[9],te[10],te[11]], [te[12],te[13],te[14],te[15]]];
    var amat = [
        [te[0],te[4],te[8],te[12]], 
        [te[1],te[5],te[9],te[13]], 
        [te[2],te[6],te[10],te[14]], 
        [te[3],te[7],te[11],te[15]]
    ];

    return amat;

    /* threejs matrix format reference
    te[0] = n11; te[4] = n12; te[8] = n13; te[12] = n14;
    te[1] = n21; te[5] = n22; te[9] = n23; te[13] = n24;
    te[2] = n31; te[6] = n32; te[10] = n33; te[14] = n34;
    te[3] = n41; te[7] = n42; te[11] = n43; te[15] = n44;
    */
}


function matrix_2Darray_to_threejs(amat) {
// conversion of 2D 4x4 matrix array to threejs format

    /* THREE r62
    var tmat = new THREE.Matrix4(
        amat[0][0], amat[0][1], amat[0][2], amat[0][3],
        amat[1][0], amat[1][1], amat[1][2], amat[1][3],
        amat[2][0], amat[2][1], amat[2][2], amat[2][3],
        amat[3][0], amat[3][1], amat[3][2], amat[3][3] 
    );
    */

    var tmat = new THREE.Matrix4().set(
        amat[0][0], amat[0][1], amat[0][2], amat[0][3],
        amat[1][0], amat[1][1], amat[1][2], amat[1][3],
        amat[2][0], amat[2][1], amat[2][2], amat[2][3],
        amat[3][0], amat[3][1], amat[3][2], amat[3][3] 
    );

    return tmat;

    /* threejs matrix format reference
    te[0] = n11; te[4] = n12; te[8] = n13; te[12] = n14;
    te[1] = n21; te[5] = n22; te[9] = n23; te[13] = n24;
    te[2] = n31; te[6] = n32; te[10] = n33; te[14] = n34;
    te[3] = n41; te[7] = n42; te[11] = n43; te[15] = n44;
    */
}

function matrix_threejs_to_mathjs(tmat) {
    // conversion of threejs 4x4 matrix to 2D array
    
        var te = tmat.elements;
        //var amat = [[te[0],te[1],te[2],te[3]], [te[4],te[5],te[6],te[7]], [te[8],te[9],te[10],te[11]], [te[12],te[13],te[14],te[15]]];
        var amat = math.matrix([
            [te[0],te[4],te[8],te[12]], 
            [te[1],te[5],te[9],te[13]], 
            [te[2],te[6],te[10],te[14]], 
            [te[3],te[7],te[11],te[15]]
        ]);
    
        return amat;
    
        /* threejs matrix format reference
        te[0] = n11; te[4] = n12; te[8] = n13; te[12] = n14;
        te[1] = n21; te[5] = n22; te[9] = n23; te[13] = n24;
        te[2] = n31; te[6] = n32; te[10] = n33; te[14] = n34;
        te[3] = n41; te[7] = n42; te[11] = n43; te[15] = n44;
        */
    }

function matrix_mathjs_to_threejs(amat) {
    // conversion of 2D 4x4 matrix array to threejs format
    
        var tmat = new THREE.Matrix4().set(
            amat.get([0,0]), amat.get([0,1]), amat.get([0,2]), amat.get([0,3]),
            amat.get([1,0]), amat.get([1,1]), amat.get([1,2]), amat.get([1,3]),
            amat.get([2,0]), amat.get([2,1]), amat.get([2,2]), amat.get([2,3]),
            amat.get([3,0]), amat.get([3,1]), amat.get([3,2]), amat.get([3,3])
        );
    
        return tmat;
    
        /* threejs matrix format reference
        te[0] = n11; te[4] = n12; te[8] = n13; te[12] = n14;
        te[1] = n21; te[5] = n22; te[9] = n23; te[13] = n24;
        te[2] = n31; te[6] = n32; te[10] = n33; te[14] = n34;
        te[3] = n41; te[7] = n42; te[11] = n43; te[15] = n44;
        */
    }

function simpleApplyMatrix(obj,mat) {
    // set the transformation matrix of a threejs object from a matrix
    // like Object3D.applyMatrix() but without multiplying with the current object transform

    // a few "don'ts"
    //joint.geom.matrixAutoUpdate = false;  // don't make this false, does not render updated of geom xform
    //joint.geom.matrix = tempmat;
    //joint.geom.applyMatrix(tempmat); // don't use applyMatrix, it mat mults with current xform

    // update object scale
    obj.position.setFromMatrixPosition(mat); // newer renaming of getFromMatrixPosition (bb31515d6b)
    // THREE r62 obj.position.getPositionFromMatrix(mat);

    // update object scale
    obj.scale.setFromMatrixScale(mat); // newer renaming of getFromMatrixScale (bb31515d6b)
    // THREE r62 obj.scale.getScaleFromMatrix(mat);

    // update object rotation as quaternion
    m1 = new THREE.Matrix4();
    m1.extractRotation(mat);
    obj.quaternion.setFromRotationMatrix( m1 );
}



