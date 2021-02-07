/* This file contains the linter configurations. */

module.exports = {
  parser: "vue-eslint-parser",
  parserOptions: {
    parser: '@typescript-eslint/parser',
    sourceType: 'module',
    ecmaVersion: 2020,
  },
  "settings": {
    "import/resolver": {
      "node": {
        "extensions": [".js", ".jsx", ".ts", ".tsx"]
      },

      "import/extensions": [
        ".js",
        ".jsx",
        ".mjs",
        ".ts",
        ".tsx",
        ".vue"
      ]
    },
  },
  extends: [
    'eslint:recommended',
    'plugin:vue/recommended',
    'plugin:@typescript-eslint/recommended',
    'airbnb-base',
  ],
  rules: {
    /*************************************** ESLint Options ***************************************/
    "array-bracket-newline": ["error", "consistent"],
    "array-bracket-spacing": ["error", "never"],
    "array-callback-return": "error",
    "array-element-newline": ["error", "consistent"],
    "block-scoped-var": "error",
    "block-spacing": "error",
    "brace-style": ["error", "stroustrup", { "allowSingleLine": false }],
    "camelcase": ["error", {
      "allow": [
        "bearing_deg",
        "completed_wps",
        "forward_back",
        "gate_width",
        "is_auton",
        "latitude_deg",
        "latitude_min",
        "left_right",
        "longitude_deg",
        "longitude_min",
        "nav_state_name",
        "num_waypoints",
        "signal_strength",
        "total_wps"
       ]
    }],
    "class-methods-use-this": "error",
    "comma-dangle": ["error", "never"],
    "comma-style": ["error", "last"],
    "consistent-return": "error",
    "curly": "error",
    "default-case": "error",
    "default-param-last": "error",
    "dot-location": ["error", "object"],
    "dot-notation": ["error", { "allowKeywords": false }],
    "eol-last": ["error", "always"],
    "eqeqeq": ["error", "smart"],
    "func-call-spacing": ["error", "never"],
    "indent": ["error", 2, {
      "SwitchCase": 1,
      "FunctionExpression": {"body": 1, "parameters": 2},
      "FunctionDeclaration": {"body": 1, "parameters": 2},
      "CallExpression": {"arguments": "first"},
      "ArrayExpression": 1,
      "ObjectExpression": 1,
      "ImportDeclaration": 1,
      "flatTernaryExpressions": false,
      "MemberExpression": 2
    }],
    "key-spacing": ["error", {"beforeColon": false, "afterColon": true, "mode": "minimum" }],
    "keyword-spacing": ["error", { "after": true }],
    "linebreak-style": ["error", "unix"],
    "lines-around-comment": [ "error", {
      "beforeLineComment": true,
      "allowBlockStart": true,
      "allowBlockEnd": true,
      "allowObjectStart": true,
      "allowObjectEnd": true,
      "allowArrayStart": true,
      "allowArrayEnd": true,
      "allowClassEnd": true,
      "allowClassStart": true,
    }],
    "lines-between-class-members": "off",
    "max-classes-per-file": "error",
    "max-len": ["error", {
      "code": 100,
      "tabWidth": 2,
      "ignoreUrls": true,
      "ignoreStrings": true,
      "ignoreComments": false,
      "ignoreRegExpLiterals": true,
      "ignoreTemplateLiterals": true,
      "ignoreTrailingComments": false
    }],
    "max-lines": ["error", 1000],
    "max-statements": ["error", 150],
    "max-statements-per-line": ["error", { "max": 1 }],
    "new-cap": "error",
    "new-parens": "error",
    "no-console": "off",
    "no-constructor-return": "error",
    "no-continue": "off",
    "no-else-return": "off",
    "no-empty-function": "error",
    "no-eq-null": "error",
    "no-extra-parens": "error",
    "no-floating-decimal": "error",
    "no-invalid-this": "error",
    "no-label-var": "error",
    "no-lonely-if": "error",
    "no-magic-numbers": ["error", {
      "detectObjects": false,
      "ignoreArrayIndexes": true,
      "ignore": [-180, -90, -1, 0, 1, 2, 60, 90, 180, 270, 360],
    }],
    "no-mixed-operators": ["error", { "allowSamePrecedence": true }],
    "no-multi-spaces": "off",
    "no-multiple-empty-lines": ["error", { "max": 2 }],
    "no-new": "error",
    "no-param-reassign": "error",
    "no-redeclare": "error",
    "no-return-assign": "error",
    "no-shadow": "error",
    "no-tabs": "error",
    "no-template-curly-in-string": "error",
    "no-trailing-spaces": "error",
    "no-undef-init": "error",
    "no-unmodified-loop-condition": "error",
    "no-unneeded-ternary": "error",
    "no-unused-expressions": "error",
    "no-use-before-define": "off",
    "no-useless-call": "error",
    "no-useless-concat": "error",
    "no-useless-return": "error",
    // "no-warning-comments": "error",
    "no-whitespace-before-property": "error",
    "nonblock-statement-body-position": ["error", "beside"],
    "object-curly-spacing": ["error", "always"],
    "one-var-declaration-per-line": ["error", "always"],
    "operator-assignment": ["error", "always"],
    "operator-linebreak": ["error", "after"],
    "padded-blocks": ["error", "never"],
    "prefer-destructuring": "off",
    "prefer-exponentiation-operator": "error",
    "prefer-object-spread": "error",
    "prefer-regex-literals": "error",
    "quote-props": ["error", "consistent-as-needed"],
    "quotes": ["error", "single"],
    "radix": ["error", "as-needed"],
    "require-atomic-updates": "error",
    "require-unicode-regexp": "error",
    "semi": "error",
    "semi-spacing": ["error", {"before": false, "after": true}],
    "semi-style": ["error", "last"],
    "space-before-function-paren": ["error", "never"],
    "space-in-parens": ["error", "never"],
    "spaced-comment": ["error", "always", { "exceptions": ["*"] }],
    "switch-colon-spacing": "error",
    "yoda": "error",


    /*********************************** AirBnB ESLint Options ************************************/
    'import/extensions': ['error', 'ignorePackages', {
      ts: 'never',
    }],


    /************************************* Vue ESLint Options *************************************/
    "vue/component-name-in-template-casing": ["error", "PascalCase", {
      "registeredComponentsOnly": true,
    }],
    "vue/eqeqeq": "error",
    "vue/html-self-closing": ["error", {
      "html": {
        "void": "never",
        "normal": "always",
        "component": "always"
      },
      "svg": "always",
      "math": "always"
    }],
    "vue/key-spacing": "error",
    "vue/match-component-file-name": "error",
    "vue/no-irregular-whitespace": "error",
    "vue/no-reserved-component-names": "error",
    "vue/no-restricted-syntax" : ["error", "VElement > VExpressionContainer CallExpression"],
    "vue/no-static-inline-styles": "error",
    "vue/no-unsupported-features": "error",
    "vue/object-curly-spacing": ["error", "always"],
    "vue/padding-line-between-blocks": ["error", "always"],
    "vue/script-indent": ["error", 2],
    "vue/singleline-html-element-content-newline": "error",
    "vue/space-infix-ops": "error",
    "vue/static-class-names-order": "error",
    "vue/v-on-function-call": ["error", "never"],


    /********************************* TypeScript ESLint Options **********************************/
    "@typescript-eslint/ban-ts-ignore": "error",
    /* In order for easy use with LCM messages that we already have, any LCM
       messages that we use in the simulator that have underscores in the name
       should be added to the ESLint rules under rule camelCase and
       @typescript-eslint/camelcase. */
    "@typescript-eslint/camelcase": ["error", {
      "allow": [
        "bearing_deg",
        "completed_wps",
        "forward_back",
        "gate_width",
        "is_auton",
        "latitude_deg",
        "latitude_min",
        "left_right",
        "longitude_deg",
        "longitude_min",
        "nav_state_name",
        "num_waypoints",
        "signal_strength",
        "total_wps"
       ]
    }],
    "@typescript-eslint/no-explicit-any": "error",
    "@typescript-eslint/no-parameter-properties": "error",
    "@typescript-eslint/no-use-before-define": "off",
    "@typescript-eslint/type-annotation-spacing": ["error", { "before": false, "after": false }],
  },
  "overrides": [
    {
      "files": ["store/modules/*State.ts"],
      "rules": {
        "no-param-reassign": "off"
      }
    },
    {
      "files": ["app.ts"],
      "rules": {
        "no-new": "off"
      }
    }
  ]
};
