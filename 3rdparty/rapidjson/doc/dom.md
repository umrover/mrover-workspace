# DOM

Document Object Model(DOM) is an in-memory representation of JSON for query and manipulation. The basic usage of DOM is described in [Tutorial](doc/tutorial.md). This section will describe some details and more advanced usages.

[TOC]

# Template {#Template}

In the tutorial,  `Value` and `Document` was used. Similarly to `std::string`, these are actually `typedef` of template classes:

~~~~~~~~~~cpp
namespace rapidjson {

template <typename Encoding, typename Allocator = MemoryPoolAllocator<> >
class GenericValue {
    // ...
};

template <typename Encoding, typename Allocator = MemoryPoolAllocator<> >
class GenericDocument : public GenericValue<Encoding, Allocator> {
    // ...
};

typedef GenericValue<UTF8<> > Value;
typedef GenericDocument<UTF8<> > Document;

} // namespace rapidjson
~~~~~~~~~~

User can customize these template parameters.

## Encoding {#Encoding}

The `Encoding` parameter specifies the encoding of JSON String value in memory. Possible options are `UTF8`, `UTF16`, `UTF32`. Note that, these 3 types are also template class. `UTF8<>` is `UTF8<char>`, which means using char to store the characters. You may refer to [Encoding](doc/encoding.md) for details.

Suppose a Windows application would query localization strings stored in JSON files. Unicode-enabled functions in Windows use UTF-16 (wide character) encoding. No matter what encoding was used in JSON files, we can store the strings in UTF-16 in memory.

~~~~~~~~~~cpp
using namespace rapidjson;

typedef GenericDocument<UTF16<> > WDocument;
typedef GenericValue<UTF16<> > WValue;

FILE* fp = fopen("localization.json", "rb"); // non-Windows use "r"

char readBuffer[256];
FileReadStream bis(fp, readBuffer, sizeof(readBuffer));

AutoUTFInputStream<unsigned, FileReadStream> eis(bis);  // wraps bis into eis

WDocument d;
d.ParseStream<0, AutoUTF<unsigned> >(eis);

const WValue locale(L"ja"); // Japanese

MessageBoxW(hWnd, d[locale].GetString(), L"Test", MB_OK);
~~~~~~~~~~

## Allocator {#Allocator}

The `Allocator` defines which allocator class is used when allocating/deallocating memory for `Document`/`Value`. `Document` owns, or references to an `Allocator` instance. On the other hand, `Value` does not do so, in order to reduce memory consumption.

The default allocator used in `GenericDocument` is `MemoryPoolAllocator`. This allocator actually allocate memory sequentially, and cannot deallocate one by one. This is very suitable when parsing a JSON into a DOM tree.

Another allocator is `CrtAllocator`, of which CRT is short for C RunTime library. This allocator simply calls the standard `malloc()`/`realloc()`/`free()`. When there is a lot of add and remove operations, this allocator may be preferred. But this allocator is far less efficient than `MemoryPoolAllocator`.

# Parsing {#Parsing}

`Document` provides several functions for parsing. In below, (1) is the fundamental function, while the others are helpers which call (1).

~~~~~~~~~~cpp
using namespace rapidjson;

// (1) Fundamental
template <unsigned parseFlags, typename SourceEncoding, typename InputStream>
GenericDocument& GenericDocument::ParseStream(InputStream& is);

// (2) Using the same Encoding for stream
template <unsigned parseFlags, typename InputStream>
GenericDocument& GenericDocument::ParseStream(InputStream& is);

// (3) Using default parse flags
template <typename InputStream>
GenericDocument& GenericDocument::ParseStream(InputStream& is);

// (4) In situ parsing
template <unsigned parseFlags>
GenericDocument& GenericDocument::ParseInsitu(Ch* str);

// (5) In situ parsing, using default parse flags
GenericDocument& GenericDocument::ParseInsitu(Ch* str);

// (6) Normal parsing of a string
template <unsigned parseFlags, typename SourceEncoding>
GenericDocument& GenericDocument::Parse(const Ch* str);

// (7) Normal parsing of a string, using same Encoding of Document
template <unsigned parseFlags>
GenericDocument& GenericDocument::Parse(const Ch* str);

// (8) Normal parsing of a string, using default parse flags
GenericDocument& GenericDocument::Parse(const Ch* str);
~~~~~~~~~~

The examples of [tutorial](doc/tutorial.md) uses (8) for normal parsing of string. The examples of [stream](doc/stream.md) uses the first three. *In situ* parsing will be described soon.

The `parseFlags` are combination of the following bit-flags:

Parse flags                   | Meaning
------------------------------|-----------------------------------
`kParseNoFlags`               | No flag is set.
`kParseDefaultFlags`          | Default parse flags. It is equal to macro `RAPIDJSON_PARSE_DEFAULT_FLAGS`, which is defined as `kParseNoFlags`.
`kParseInsituFlag`            | In-situ(destructive) parsing.
`kParseValidateEncodingFlag`  | Validate encoding of JSON strings.
`kParseIterativeFlag`         | Iterative(constant complexity in terms of function call stack size) parsing.
`kParseStopWhenDoneFlag`      | After parsing a complete JSON root from stream, stop further processing the rest of stream. When this flag is used, parser will not generate `kParseErrorDocumentRootNotSingular` error. Using this flag for parsing multiple JSONs in the same stream.
`kParseFullPrecisionFlag`     | Parse number in full precision (slower). If this flag is not set, the normal precision (faster) is used. Normal precision has maximum 3 [ULP](http://en.wikipedia.org/wiki/Unit_in_the_last_place) error.
`kParseCommentsFlag`          | Allow one-line `// ...` and multi-line `/* ... */` comments (relaxed JSON syntax).
`kParseNumbersAsStringsFlag`  | Parse numerical type values as strings.
`kParseTrailingCommasFlag`    | Allow trailing commas at the end of objects and arrays (relaxed JSON syntax).
`kParseNanAndInfFlag`         | Allow parsing `NaN`, `Inf`, `Infinity`, `-Inf` and `-Infinity` as `double` values (relaxed JSON syntax).

By using a non-type template parameter, instead of a function parameter, C++ compiler can generate code which is optimized for specified combinations, improving speed, and reducing code size (if only using a single specialization). The downside is the flags needed to be determined in compile-time.

The `SourceEncoding` parameter defines what encoding is in the stream. This can be differed to the `Encoding` of the `Document`. See [Transcoding and Validation](#TranscodingAndValidation) section for details.

And the `InputStream` is type of input stream.

## Parse Error {#ParseError}

When the parse processing succeeded, the `Document` contains the parse results. When there is an error, the original DOM is *unchanged*. And the error state of parsing can be obtained by `bool HasParseError()`,  `ParseErrorCode GetParseError()` and `size_t GetErrorOffset()`.

Parse Error Code                            | Description
--------------------------------------------|---------------------------------------------------
`kParseErrorNone`                           | No error.
`kParseErrorDocumentEmpty`                  | The document is empty.
`kParseErrorDocumentRootNotSingular`        | The document root must not follow by other values.
`kParseErrorValueInvalid`                   | Invalid value.
`kParseErrorObjectMissName`                 | Missing a name for object member.
`kParseErrorObjectMissColon`                | Missing a colon after a name of object member.
`kParseErrorObjectMissCommaOrCurlyBracket`  | Missing a comma or `}` after an object member.
`kParseErrorArrayMissCommaOrSquareBracket`  | Missing a comma or `]` after an array element.
`kParseErrorStringUnicodeEscapeInvalidHex`  | Incorrect hex digit after `\\u` escape in string.
`kParseErrorStringUnicodeSurrogateInvalid`  | The surrogate pair in string is invalid.
`kParseErrorStringEscapeInvalid`            | Invalid escape character in string.
`kParseErrorStringMissQuotationMark`        | Missing a closing quotation mark in string.
`kParseErrorStringInvalidEncoding`          | Invalid encoding in string.
`kParseErrorNumberTooBig`                   | Number too big to be stored in `double`.
`kParseErrorNumberMissFraction`             | Miss fraction part in number.
`kParseErrorNumberMissExponent`             | Miss exponent in number.

The offset of error is defined as