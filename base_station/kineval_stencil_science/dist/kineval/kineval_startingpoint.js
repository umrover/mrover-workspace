/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics 
     in HTML5/JavaScript and threejs
     
     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

     The starting point routines below are to show how rigid bodies are 
       displayed and transformed interactively based on user controls,
       as well as provide a coarse guide to coding in JavaScript.
       Objects will be evenly distributed along the world x-axis and animated
       to bounce in the rhythm of a sine wave.  Additional translational 
       offsets will be controlled interactive through key controls

*/


kineval.startingPlaceholderInit = function startingPlaceholderInit() {

    /* startingPlaceholderInit() is invoked only once, from my myInit() in 
         home.html.  This function initializes the starting point example 
         animation data structures and gives some insights-by-example about 
         JavaScript. 
         Note: startingPlaceholderInit() must be called to ensure the textbar 
         object is created
    */

    // READ THIS JS TUTORIAL-BY-EXAMPLE FUNCTION (in file XXX)
    
    kineval.tutorialJavaScript();

    // READ THIS OVERVIEW OF KINEVAL ROBOT OBJECT (in file XXX)

    //kineval.printRobotInfo();

    // CREATE ANIMATION VARIABLES 

    // assign relevant variables for the starting point animation
    var spacingLocal = 0.9;  // variables declared with "var" are local 
    spacingGlobal = 0.9;  // variables declared with "var" are global
    waveAmplitude = 1.0;  // what does this do?
    waveFrequency = 0.31;  // could be interesting to modify this variable
    offsetVertical = 0.0;  // this could be useful later
    jitterRadius = 0.0;  // and this too


}

function startingPlaceholderAnimate() {

    /* startingPlaceholderAnimate() will be continuously, from my myAnimate() 
         in home.html.  This function updates the animation loop for the 
         starting point example.
    */

    // get the current in deciseconds from the global Date object
    var cur_time = Date.now()/100;

    // CREATE TRANSFORMATION MATRIX

    // jsmat is the matrix data structure used to separately transform each 
    //   3D object to a specific location in the world. We will represent 
    //   matrices with index notation, such that matrix[row][column] indexes 
    //   into elements of the matrix

    var jsmat = [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1] 
    ];

    // TRANSLATE ROBOT LINKS

    // Iterate over each link of the robot independently to compute a 
    //   translation matrix transform that will set the joint object 
    //   geometry to its proper place in 3D.

    // Set the start the location of the first object along the x-axis.
    //   Note: Object.key(object) is a rough way to get number of keys in an 
    //   object, in this case the number of joints in the robot.joints object

    jsmat[0][3] = -Object.keys(robot.links).length*spacingGlobal/2;

    // iteration loop over each link named in the robot.links array
    for (x in robot.links) {

        // add spacing offset on x-axis for translation of next link geometry
        jsmat[0][3] += spacingGlobal;

        // set object y-axis position
        if (1 == 1) { // bouncing animation set to absolute value of sine wave 
            jsmat[1][3] = waveAmplitude * Math.abs(Math.sin(((cur_time+jsmat[0][3])*waveFrequency))) + offsetVertical + jitterRadius*Math.random();
        }
        else { // older animation for jittering objects
            jsmat[1][3] = offsetVertical+Math.random()*jitterRadius;
        }

        // set object z-axis position
        jsmat[2][3] = Math.random()*jitterRadius;

        var copy = new THREE.Matrix4().set( jsmat[0][0], jsmat[0][1], jsmat[0][2], jsmat[0][3],
                                            jsmat[1][0], jsmat[1][1], jsmat[1][2], jsmat[1][3],
                                            jsmat[2][0], jsmat[2][1], jsmat[2][2], jsmat[2][3],
                                            jsmat[3][0], jsmat[3][1], jsmat[3][2], jsmat[3][3] )

        // apply matrix to transform the object to its proper pose in 3D
        robot.links[x].xform = copy;
    } 

    // TRANSLATE ROBOT JOINTS

    // Iterate over each joint of the robot independently to set the poses of
    // each displayed joint geometry 

    jsmat[0][3] = -Object.keys(robot.joints).length*spacingGlobal/2;
    for (x in robot.joints) {
        jsmat[0][3] += spacingGlobal;
        if (1 == 1) { // bouncing animation set to absolute value of sine wave 
            jsmat[1][3] = waveAmplitude * Math.abs(Math.sin(((cur_time+jsmat[0][3])*waveFrequency))) + offsetVertical + jitterRadius*Math.random();
        }
        else { // older animation for jittering objects
            jsmat[1][3] = offsetVertical+Math.random()*jitterRadius;
        }
        jsmat[2][3] = Math.random()*jitterRadius;

        var copy = new THREE.Matrix4().set( jsmat[0][0], jsmat[0][1], jsmat[0][2], jsmat[0][3],
            jsmat[1][0], jsmat[1][1], jsmat[1][2], jsmat[1][3],
            jsmat[2][0], jsmat[2][1], jsmat[2][2], jsmat[2][3],
            jsmat[3][0], jsmat[3][1], jsmat[3][2], jsmat[3][3] )

        // apply matrix transform to joint geometry pose
        robot.joints[x].xform = copy;
    } 

} 


kineval.tutorialJavaScript = function tutorialJSCoding() {

    // COARSE JAVASCRIPT CODE-BY-EXAMPLE TUTORIAL: Variables and Objects

    /* JavaScript objects are key/value pairs.  Object keys are properties of
         the object that index into values, stored as variables.  JavaScript
         variables can be of a primitive type (e.g., "number", "string") or
         of an object type 
    */
    myObject = {};  // objects can be created with braces

    // create object property with an assignment
    myObject.university = "Michigan"; // this variable is of type "object"

    // equivalent to my.object.department = "EECS";
    myObject["department"] = "EECS"; // this variable is of type "string"

    myObject.course_number = 367;  // this variable is of type "number"

    // this variable is a number with value 367002; "+" operator adds numbers
    //   Note: the "pow" method of the "Math" object is invoked
    myObject.course_number = myObject.course_number*Math.pow(10,3) + 2;  

    // this variable is a string; "+" operator concatenates strings 
    myObject.course_number = Math.floor(myObject.course_number/1000).toString() + "-" + "00" + (myObject.course_number%1000).toString();  

    /* A property of an object can be index using the key within brackets or 
       after a period.  However, myObject[string_containing_the_word_subject] 
       is not equivalent to  myObject.string_containing_the_word_subject
    */
    var string_containing_the_word_subject = "subject";
    myObject[string_containing_the_word_subject] = "robotics"; 
    myObject.string_containing_the_word_subject = "an irrelevant property"; 

    // JAVASCRIPT TUTORIAL: References to objects

    /* JavaScript allows access to variables that are references to JavaScript 
         objects, but not the actual object data structure itself.  As such,
         memory management is handled automatically, which includes garbage 
         collection.  Another convenience is that a reference to an object
         can be copied to another variable through a simple assignment.
         However, such assignments can lead to big misunderstandings if 
         deep copying of the object is assumed.
       Note: the typeof operator returns the type of a variable, and also 
         returns "undefined" when a variable does not exist.  
       Note: the equivalence boolean operators are "===" for equals and "!==" 
         for not equals.  "==" and "!=" also work, but with less predictability
    */

    // if copiedObject does not already exist
    if (typeof copiedObject === 'undefined') {  

        console.log(myObject);  // check it out on the console  
        console.log(JSON.stringify(myObject));  // same thing as a string  

        /* IMPORTANT!!! 
             objects are not "deep" copied upon assignment.  This assignment 
             will only copy a reference to the same object in memory.  Thus, 
             modifying a property of copiedObject will also modify myObject.
        */

        // these assignments will modify the data structure underlying both
        // copiedObject and myObject
        copiedObject = myObject;
        copiedObject.subject = "autonomous_robotics";  

        /* VERIFY
             - What does myObject.subject report on the console?
             - Inspect "copiedObject.subject" in the console and compare.
        */
    }

    // using our knowledge of reference, we can use "autorobObject" instead of
    //   my Object
    autorobObject = myObject;

    // JAVASCRIPT TUTORIAL: Arrays

    /* JavaScript arrays are a special instance of an object data structure
       with numeric keys for indexing and a "length" property 
    */
    var emptyArray = [];  // create an object as an empty array
    myArray = [8, 6, 7, 5, 3, 0, 9]; // create a 7-element array

    // output each element to the console
    var i;  // local variable serving as an iterator through an array
    for (i=0;i<myArray.length;i++) {
        console.log(myArray[i]);  
    }

    // JAVASCRIPT TUTORIAL: Dynamic typing

    /* JavaScript does not enforce strict typing of variables.  Thus, the type
         of a variable can changed based on the value last assigned
    /*
    // replace the sixth element, currently a number, with a string
    myArray[6] = 'ni-i-i-ine';  

    // store a reference to myArray as a property in autorobObject
    autorobObject.phoneArray = myArray;

    // JAVASCRIPT TUTORIAL: The "window" object and global variables

    /* The "window" object maintains the global scope for all variables. The 
         browser maintains a sandboxed run-time environment for each 
         individual web page being displayed.  All global variables are 
         properties of the window object, including myObject.  Other variables
         are scoped locally within a function, object, etc.
    */

    window.autorobObject.instructor = "ocj"; // myObject.instructor == "ocj"

    // JAVASCRIPT TUTORIAL: Functions 

    /* JavaScript functions are a special instance of object that can be 
         invoked with a function a call.  Functions are declared are declared
         using the "function" reserved word. A reference to a function
         object can be assigned by reference to another variable.
    */

    // declare a function to print myObject to the console 
    function myFunction(inputObject) {

        // Object.keys() method returns an array of top-level keys in an object
        myObjectKeys = Object.keys(inputObject);

        // output each key/value of myObject to console as formatted strings
        for (i=0;i<myObjectKeys.length;i++) {
            var x = myObjectKeys[i];
            console.log("AutoRob " + x + " is " + inputObject[x]);
        }

        /* the reserved variable "this" is a reference based on the context
             by which myFunction is invoked.  "this" has been assigned to
             an object in the scope of a longer discussion. 
        */ 
        //console.log(this);
    }

    // assign myFunction() function as a property of the autorobObject object
    autorobObject.printCourseInfo = myFunction;

    // invoke myFunction as a member of the autorobObject function
    window.myObject.printCourseInfo(window.autorobObject);


    // JAVASCRIPT TUTORIAL: The "document" object and the Document Object Model

    /* JavaScript can dynamically modify HTML documents running in a browser
       window environment.  The global variable "document" maintains the 
       Document Object Model (DOM) that stores the HTML document of the web 
       page in a tree data structure.  The document object can be accessed
       from JavaScript to dynamically alter the DOM, and thus the run-time
       version of the web page.  The document object is a property of window.
    */

    // create an empty div HTML element in the DOM
    textbar = document.createElement('div'); 

    // set element style (CSS) parameters of the created element
    textbar.style.position = 'absolute'; 
    textbar.style.top = 10 + 'px';
    textbar.style.left = 10 + 'px';
    textbar.style["font-family"] = "Monospace";
    textbar.style.color = "#00ff00";
    textbar.style.height = 40;
    // Note: window.innerWidth current width of the window viewport 
    textbar.style.width = window.innerWidth-10; 
    
    // if you do not see the textbar, try uncommenting this
    //textbar.style.zIndex = 1; 

    // assign an identifier to the created div element
    textbar.id = "textbar";

    // attach the div element to the DOM under the body of the HTML document
    document.body.appendChild(textbar);  

    posText = document.createElement('div');
    angText = document.createElement('div');
    textbar.appendChild(posText);
    textbar.appendChild(angText);
    posText.id = "posText";
    angText.id = "angText";
    posRef = document.getElementById("posText");
    angRef = document.getElementById("angText");
    

    /* The document.getElementById() method returns a reference to an object
         in the DOM that has the id specified in the input parameter
    */

    // get a reference to the textbar element in the DOM
    textbarReference = document.getElementById("textbar");
}


kineval.printRobotInfo = function printRobotStuff() {   

    // PRINT MISCELLANEOUS ROBOT INFORMATION

    console.log("printing miscellaneous robot information");

    // uncomment line below to print out initial starting robot data object
    //   in the console (click it in the console to inspect)

    console.log(robot);  

    // uncomment line below to print out initial starting robot data object
    //   in the console, but this time as a string 

    console.log(JSON.stringify(robot));  

    // print the names of all the links of the robot in the console 

    console.log(robot.links);  

    // print the names of all the joints of the robot

    console.log(robot.joints);  

    // print the name of the base link the robot in the console

    console.log(robot.links[robot.base].name); // the same as robot.base

    // print the current matrix transform of the robot base link 

    console.log(robot.links[robot.base].xform); 

    // print the motor axis of the first child joint of the robot base link 

    console.log(robot.joints[robot.links[robot.base].children[0]].axis); 

}
