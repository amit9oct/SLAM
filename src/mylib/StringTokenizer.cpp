#include "StringTokenizer.h"

StringTokenizer::StringTokenizer(char *line, char *delim) {
  char *bptr, *eptr, *dptr;

  nTokens = 0;
  first = NULL;
  last = NULL;
  if (!line) {
    printf("StringTokenizer::StringTokenizer : no hay linea\n");
    return;
  }
  if (!delim) {
    printf("StringTokenizer::StringTokenizer : no hay linea\n");
    return;
  }
  bptr = line;
  eptr = bptr;
  while ( 1 ) {
    char *aux;
    if (*eptr) { 
      // Si hay algun caracter en la linea miro si coincide con el delim
      dptr = delim;
      aux = eptr;
      //      printf("comparando <%c> y <%c>\n", *eptr, *dptr);
      while (*aux && *dptr && *aux==*dptr) { 
	// Si coincide busco el final de delim para ver si es un separador
	//	printf("<%c == %c>\n", *eptr, *dptr);
	aux++;
	dptr++;
      }
      //      printf("Delimitador : aux - eptr = %d\n", aux-eptr);
      if (!*dptr) { 	
	// Si hemos reconocido un delimitador empezando en eptr nos quedamos con el token anterior
	/*
	printf("Hemos encontrado un delim que empezaba en bptr y termina en eptr:\n");
	printf("Longitud del token = %d\n", eptr-bptr);
	*/
	addToken(bptr, eptr-bptr);
	/*
	for ( char *ptr = bptr ; ptr != eptr ; ptr++) {
	  printf("[%c]", *ptr);
	}
	printf("\n");
	*/
	eptr = aux;
	bptr = eptr;
      } else {
	eptr++;
      }
    } else {
      //      if (eptr-bptr+1 > 0) printf("Al final hay %dlentras\n", eptr-bptr+1);
      addToken(bptr, eptr-bptr);
      break;
    }
  }
  next = first;
}

int StringTokenizer::addToken(char *token, int size) {
  Nodo *n;

  if (!size) return -1;
  n = (Nodo *)malloc(sizeof(Nodo));
  n -> token = (char *)malloc(sizeof(char)*(size+1));
  strncpy(n -> token, token, size);
  n -> token[size] = '\0';
  if (first) last -> next = n;
  else first = n;
  last = n;
  n -> next = NULL;
  nTokens++;
  return 1;
}

StringTokenizer::~StringTokenizer() {
  Nodo *n;

  if ( first ) {
    n = first;
    first = first -> next;
    free (n -> token);
    free (n);
  }
}

char *StringTokenizer::nextToken(char *delim) {
  char *ptr = NULL;
  if ( next ) {
    ptr = next -> token;
    next = next -> next;
  }
  return ptr;
}


int StringTokenizer::countTokens() {
  return nTokens;
}

int StringTokenizer::hasMoreTokens() {
  if (first) return (first -> next!= NULL);
  else return 0;
}
