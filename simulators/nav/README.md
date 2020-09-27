# <img src="src/static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Simulator

## Table of Contents
[Project Overview](#project-overview)
[Running the Simulator](#running-the-simulator)
[Code Overview](#code-overview)
[Recommended Knowledge](#recommended-knowledge)
[Styling](#styling)
[Related Documents](#related-documents)

---

<!----------------------------- Project Overview ----------------------------->
## Project Overview
The Navigation Simulator (also referred to as `simulators/nav`) is a web application that allows for local testing of the MRover Autonomous Navigation system (Nav). The main use is for isolated testing of Nav, but there are several features that allow for integration testing of Nav with the other Autonomy systems (i.e. Localization and Perception). This simulator is not designed to provide thorough testing of the other Autonomy systems.

---

<!-------------------------- Running the simulator -------------------------->
## Running the Simulator
In order to run the simulator you need to build and run `simulators/nav`. In order to test the Navigation code with the simulator you need to also build and run `jetson/nav` and `lcm_bridge/server`. You can also choose to use `jetson/cv` and/or `jetson/filter` if you want to test nav with either of these systems which is a good way to do some manual integration testing. If you do this make sure to also uncheck the corresponding Simulate Perception and/or Simulator Localization buttons.

As a reminder, you can build a program by running the command `./jarvis build <service name>` (e.g. `./jarvis build simulator/nav`) and you can run a program by running the command `./jarvis exec <service name>` (e.g. `./jarvis exec simulator/nav`). Note that you may be able to use `jarvis` instead of `./jarvis` on your local machine.

---

<!------------------------------- Code Overview ------------------------------>
## Code Overview
This sections gives descriptions of the various directories and files in the project in order to give a better understanding of how the code is organized. This section is not complete, however you can find the remaining details in other `README` files across this project. Ultimately, you can get a comprehensive view of the project through this document, the [`src` `README`](./src/README.md), the [`components` `README`](./src/components/README.md), and . Keep reading for more details.

#### `src`
The `src` directory contains all the source code for the project. For more information on the code in this directory, see [`src/README.md`](./src/README.md).

#### `package.json`
The `package.json` file specifies the dependencies which are packages to be downloaded for later use (e.g. imports, etc.). Amongst other things it also gives the command to run when building the project.

#### `project.ini`
`project.ini` is used by `jarvis`. It specifies the programming language used, any dependencies, and what port the program will use (i.e. localhost:8010).

#### `tsconfig.json`
`tsconfig.json` specifies various typescript compiler options.

#### `webpack.config.dev.js`
`webpack.config.dev.js` specifies how to build the project. For example it says our entry point or where the root of our project is and it says to run the linter before compiling. See this [Beginner's Guide to Webpack](https://medium.com/javascript-training/beginner-s-guide-to-webpack-b1f1a3638460){target="_blank" rel="noopener noreferrer"} for more information on Webpack.

---

<!--------------------------- Recommended Knowledge -------------------------->
## Recommended Knowledge
This section lists topics that one should be familiar with in order to effectively contribute to this project. For each topic, there is also linked documentation, tutorials, and helpful resources to aid in the learning of these topics. As better tutorials and documentation are found and as features are added to this project, this list of topics and corresponding helpful links should be expanded. The more information we have here, the better, so please feel free to add resources that you found helpful (just be sure to give a description of what you are linking to)!

It is important to keep in mind that we are all here on MRover to learn and we don't expect you to know everything when you join the team (or when you start working on a new project). Therefore, we encourage you to take some time to do learn about the topics that we use in our projects even if it means time not directly contributing to building the rover.

### Vue
Vue is the framework that we use to build the UI of the simulator.
The [Vue Documentation](https://vuejs.org/v2/guide/){target="_blank" rel="noopener noreferrer"} which is a great starting point for learning Vue but keep in mind that we do a lot of things on top of just using Vue. It has lots of videos throughout so check this out if you're a visual learner.

##### Vuex
Vuex is how we manage state throughout the simulator.
The [Vuex Documenatation](https://vuex.vuejs.org){target="_blank" rel="noopener noreferrer"} is a great resource and be sure to check out the Vuex Explained Visually video.

##### `vue-property-decorator`
We use the [`vue-property-decorator`](https://github.com/kaorun343/vue-property-decorator){target="_blank" rel="noopener noreferrer"} library throughout for many basic functionalities from creating a component to creating references to HTML elements. Check out the documentation for more details.

##### `vuex-class`
We use the [`vuex-class`](https://github.com/ktsn/vuex-class){target="_blank" rel="noopener noreferrer"} library in many components to access our Vuex store (through getters and mutations). Check out the documentation for more details.

##### Tutorials
We also recommend searching for and completing some online tutorials for Vue. Remember to add tutorials that you found to be useful with a short description to this section!

* [Scrimba](https://scrimba.com/g/gvuedocs){target="_blank" rel="noopener noreferrer"} is the tutorial that the Vue documentation recommends.

##### Helpful References
* We highly recommend you use the [Vue DevTools extension for Chrome](https://chrome.google.com/webstore/detail/vuejs-devtools/nhdogjmejiglipccpnnnanhbledajbpd?hl=en){target="_blank" rel="noopener noreferrer"} if you are developing using the Chrome browser. If you are developing with another browser, we suggest looking into similar tools. You can easily use this to visualize the breakdown of components. Additionally, we suggest you use more than one browser when developing (especially when making HTML/CSS changes) because many things don't look the same on different browsers.

#

### JavaScript
JavaScript is the programming language we use to implement what the simulator does and how it does it. Okay, that's a lie. We actually use TypeScript but JavaScript could be used to get the same exact effect.
Here is the [W3Schools JavaScript documentation](https://www.w3schools.com/js/){target="_blank" rel="noopener noreferrer"} which details many aspects of JavaScript. While we actually use TypeScript, we highly recommend starting with learning JavaScript as it is a bit easier to work with because it's more forgiving (which also means it's more prone to bugs).

##### Tutorials
We also recommend searching for and completing some online tutorials for JavaScript. Remember to add tutorials that you found to be useful with a short description to this section!

* The [W3Schools JavaScript Documentation](https://www.w3schools.com/js/){target="_blank" rel="noopener noreferrer"} is also a tutorial!

##### Helpful References
* [Top Ten Pitfalls of JS](https://www.toptal.com/javascript/10-most-common-javascript-mistakes){target="_blank" rel="noopener noreferrer"}
* [3 things you didn’t know about the forEach loop in JS](https://medium.com/front-end-weekly/3-things-you-didnt-know-about-the-foreach-loop-in-js-ff02cec465b1){target="_blank" rel="noopener noreferrer"}

#

### TypeScript
TypeScript is the programming language we use to implement what the simulator does and how it does it. Yes, for real this time.
[TypeScript](https://www.typescriptlang.org){target="_blank" rel="noopener noreferrer"} is a programming language that extends JavaScript which adds static typing. The TypeScript documentation has a ton of information on the langauge.


##### Tutorials
We also recommend searching for and completing some online tutorials for HTML. Remember to add tutorials that you found to be useful with a short description to this section!

* [TutorialsTeacher](https://www.tutorialsteacher.com/typescript){target="_blank" rel="noopener noreferrer"} is a good tutorial but does seem to require some (very little) prior knowledge. If nothing else, it is a good source of documentation.

##### Helpful References
* [What is JavaScript?](https://developer.mozilla.org/en-US/docs/Learn/JavaScript/First_steps/What_is_JavaScript){target="_blank" rel="noopener noreferrer"}

#

### HTML
HTML is the markup language we use to implement what the simulator displays on the page (the contents of the webpage).
Here is the [W3Schools HTML documentation](https://www.w3schools.com/html/){target="_blank" rel="noopener noreferrer"} which details many aspects of HTML.

##### Tutorials
We also recommend searching for and completing some online tutorials for HTML. Remember to add tutorials that you found to be useful with a short description to this section!

* The [W3Schools HTML Documentation](https://www.w3schools.com/html/){target="_blank" rel="noopener noreferrer"} is also a tutorial!

##### Helpful References
* [Flexbox](https://internetingishard.com/html-and-css/flexbox/){target="_blank" rel="noopener noreferrer"}: We use flexboxes all over the simulator. They allow us to do a ton of the dynamic movement of components as you resize the window.
* [HTML + CSS is turing complete](https://stackoverflow.com/questions/2497146/is-css-turing-complete).

#

### CSS
CSS is the styling language we use to implement what the simulator looks like.
Here is the [W3Schools CSS documentation](https://www.w3schools.com/css/){target="_blank" rel="noopener noreferrer"} which details many aspects of CSS.

##### Tutorials
We also recommend searching for and completing some online tutorials for CSS. Remember to add tutorials that you found to be useful with a short description to this section!

* The [W3Schools CSS Documentation](https://www.w3schools.com/css/){target="_blank" rel="noopener noreferrer"} is also a tutorial!

##### Helpful References
* The [difference between `.class-A .class-B` and `.class-A.class-B`](https://stackoverflow.com/questions/10036156/whats-the-difference-between-css-classes-foo-bar-without-space-and-foo-bar){target="_blank" rel="noopener noreferrer"}.

---

<!---------------------------------- Styling --------------------------------->
## Styling
Yes. We know. The linters are super strict and annoying. But we do this for a reason. We want to minimize the chance of developers making common JavaScript mistakes and we want to ensure that our code looks consistent so that new developers can more easily understand what is being done in the code.

Documentation for the [ESLint rules](https://eslint.org/docs/rules/){target="_blank" rel="noopener noreferrer"} describes what each ESLint rule does. Similary, so does the [Vue ESLint rules](https://github.com/vuejs/eslint-plugin-vue/tree/master/docs/rules){target="_blank" rel="noopener noreferrer"} for the `vue/*` rules and the [TypeScript ESLint rules](https://github.com/typescript-eslint/typescript-eslint/tree/master/packages/eslint-plugin/docs/rules){target="_blank" rel="noopener noreferrer"} for the `@typescript/*` rules.

Below is a list of styling guidelines that we ask you to adhere to when developing. These are things that we could not have the linter do for us. We also ask that as a reviewer, you enforece the following guidelines.

* Do not change the linting rules without good cause. By "good cause" we mean that it would be a bad coding practice to find another solution that satisfies the linter.
* Do not use `disable-eslint` or `@ts-ignore` unless it is truly necessary (which it almost never is. We concede that we have done the former (and at one point in development did the latter) in the `NavSimulator` component, but they are the exception, not the standard.
    * The reason we used `disable-eslint` is because we were interacting with `LCM` messages (the rest of the project uses the data in the `LCM` messages and has the same structure for the data but that's just for convenience). `LCM` messages are required to have a `type` attribute in order to specify the LCM type (e.g. TargetList or Odometry). An alternative would have been to create additional `Types` (see [`src/utils/types.ts`](./src/utils/types.ts)) but we felt this would have added unnecessary confusion and the `type` attribute is a constant which is not something we could have encoded in a `Type` (i.e. the `type` attribute of the `Odometry` `LCM` message is always `Odometry` but we can't enforce that outside of getting runtime errors from the `LCMBridge`).
    * The reason we temporarily used `@ts-ignore` is because we are interacting with the `LCMBridge` using the `LCMBridgeClient`. `LCMBridge` is a program that allows us to use `LCM` in our JavaScript programs. A quick aside: `LCM` is not a common library for JavaScript like it is for C++ and Python so we had to build a custom library to do it for use (see [`/lcm_bridge`](/lcm_bridge)). The `LCMBridge` is composed of `LCMBridgeClient`s and an `LCMBridgeServer`. JavaScript programs create an instance of `LCMBridgeClient` in order to communicate with the instance of the `LCMBridgeServer` which you run using `jarvis`. If you don't `jarvis exec lcm_bridge/server` then the JavaScript programs will not be able to communicate with the other programs in our code base and vice versa (e.g. `jetson/nav` won't be able to communicate with `simulators/nav`). However, non-JavaScript programs will be able to communicate with each other without an instance of `LCMBridgeServer` (e.g. `jetson/filter` will be able to communicate with `jetson/nav`). Even though we didn't end up using `@ts-ignore`, we are noting this because the rest is still true and is something to be aware of. However, you should still stay away from `@ts-ignore`.
* Keep the maximum line length to 80 characters or less for comments. The only comments that are allowed to be longer are the comments that denote the different sections of the code (e.g. `<!-- Template -->` or `/* Private Methods */`). This practice allows us to easily spot the different sections of the code when we are quickly scanning the code. An easy way to do this is to have your text editor display a line at 80 characters and quickly check that no code goes beyond this line.
* The linter usually catches lines that are over 100 characters but we have found some cases where it doesn't so still make a quick check for this. Similar to above, an easy way to do this is to have your text editor display a line at 100 characters and quickly check that no code goes beyond this line. You can also use the section header comments as a guide since they are at 100 characters.
* The major sections of Vue files should be the following in order (leaving out ones that are empty in a file):
    * `Template`
    * `Script`
    * `Scoped Style`
    * `Style` (should only be used in [`src/App.vue`](./src/App.vue))
* Within the `Script` section or within `.ts` files should be the following in order (leaving out ones that are empty in or don't apply to a file):
    * `Constants`
    * `Types`
    * `Props`
    * `Vuex Getters`
    * `Vuex Mutations`
    * `HTML Refs`
    * `Staic Members`
    * `Private Members`
    * `Public Members`
    * `Local Getters/Setters`
    * `Watchers`
    * `Emittin Private Methods`
    * `Private Methods`
    * `Public Methods`
    * `Vue Life Cycle`
* Functions should be ordered alphabetically within sections and members should be grouped logically.
* All methods should have be commented and all members (unless painfully obvious) should be commented (with units where applicable).
* All comments should use be block style (i.e. `/* comment */`) rather than inline style (i.e. `// comment`). This allows for consistency and it helps prevent quick debugging comments from making it into our codebase. Inline style comments are not prohibited by our linter (because the goal of linting is not to slow down development) and are welcomed in the development cycle (just not in the final commit). The linter also disallows comments with phrases like "todo" and "fixme". We recognize that this is a part of normal development so feel free to comment out `no-warning-comments` in [`src/eslint.js`](./src/eslint.js) to disable this (but make sure to comment it back in before making your final commit intended to be merged into master).
* In order for easy interface with the `LCM` messages that we use, any `LCM` messages that we use in the sim that have underscores in the name should be added to the ESLint rules ([`src/eslint.js`](./src/eslint.js)) under rule `camelcase` and `@typescript-eslint/camelcase`.
* The [Vue style guide](https://vuejs.org/v2/style-guide/){target="_blank" rel="noopener noreferrer"} should be reviewed and followed. If any rules we have in [`src/eslint.js`](./src/eslint.js) conflict with it in the future, our rules should be modified to match the Vue style guide.
* For CSS, scoped styling should be used in the components they apply to and  styling that applies to the entire project should be placed in [`src/App.vue`](./src/App.vue).
* For CSS, order styling block alphabetically. This makes it easier to find relevant blocks when checking, changing, and adding styling. Despite this, know that there are styling blocks that apply to multiple classes/tags or nested classes/tags and these may not appear where you'd necessarily expect them so be sure to do a complete search for a block before creating a duplicate block that could end up with negating styling in the other block (or vice versa).

---

<!----------------------------- Related Documents ---------------------------->
## Related Documents
* [`src` `README`](./src/README.md)
* [`components` `README`](./src/components/README.md)
