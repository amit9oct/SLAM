#ifndef __StringTokenizer_H__
#define __StringTokenizer_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
* @brief Splits a string in substrings separated by a delimiter
*
*/
class StringTokenizer {
  
private:
  /// String tokenizer node
  typedef struct Nodo {
    struct Nodo *next;
    char *token;
  } Nodo;
  
  int nTokens;
  Nodo *first;
  Nodo *last;
  Nodo *next;
  int addToken(char *token, int size);

public:
  StringTokenizer(char *line, char *delim = " ");
  virtual ~StringTokenizer();
  char *nextToken(char *delim = " ");
  int countTokens();
  int hasMoreTokens();
};

#endif






