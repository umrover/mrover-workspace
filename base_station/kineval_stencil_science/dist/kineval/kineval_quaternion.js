//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
function quaternion_from_axisangle(angle,u) {
    return [ Math.cos(angle/2), u[0]*Math.sin(angle/2), u[1]*Math.sin(angle/2), u[2]*Math.sin(angle/2)];
}


function   quaternion_normalize(q1){
    var mag = 0;
    var q = [];
    for (i=0;i<q.length;i++){
        mag = mag + q1[i]*q1[i];
    }
    mag = Math.sqrt(mag);
    for (i=0;i<q1.length;i++){
        if (Math.abs(mag)<Number.EPSILON){
            q[i] = q1[i];
        } else {
            q[i]=q1[i]/mag;
        }
    }
    return q;

}

function   quaternion_to_rotation_matrix(q){
    var m = generate_identity(4);
    m[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    m[0][1] = 2*( q[1]*q[2] - q[0]*q[3] );
    m[0][2] = 2*( q[0]*q[2] + q[1]*q[3] );
    m[1][0] = 2*( q[1]*q[2] + q[0]*q[3] );
    m[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    m[1][2] = 2*( q[3]*q[2] - q[0]*q[1] );
    m[2][0] = 2*( q[1]*q[3] - q[0]*q[2] );
    m[2][1] = 2*( q[1]*q[0] + q[2]*q[3] );
    m[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    return m;

}
function   quaternion_multiply(q1,q2){
    var q = new Array(q1.length);
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q[3] = q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] + q1[3]*q2[0];
    return q;

}
    