
#ifndef CASADI_LINEARSOLVER_LAPACKLU_EXPORT_H
#define CASADI_LINEARSOLVER_LAPACKLU_EXPORT_H

#ifdef CASADI_LINEARSOLVER_LAPACKLU_STATIC_DEFINE
#  define CASADI_LINEARSOLVER_LAPACKLU_EXPORT
#  define CASADI_LINEARSOLVER_LAPACKLU_NO_EXPORT
#else
#  ifndef CASADI_LINEARSOLVER_LAPACKLU_EXPORT
#    ifdef casadi_linearsolver_lapacklu_EXPORTS
        /* We are building this library */
#      define CASADI_LINEARSOLVER_LAPACKLU_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CASADI_LINEARSOLVER_LAPACKLU_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CASADI_LINEARSOLVER_LAPACKLU_NO_EXPORT
#    define CASADI_LINEARSOLVER_LAPACKLU_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CASADI_LINEARSOLVER_LAPACKLU_DEPRECATED
#  define CASADI_LINEARSOLVER_LAPACKLU_DEPRECATED __attribute__ ((__deprecated__))
#  define CASADI_LINEARSOLVER_LAPACKLU_DEPRECATED_EXPORT CASADI_LINEARSOLVER_LAPACKLU_EXPORT __attribute__ ((__deprecated__))
#  define CASADI_LINEARSOLVER_LAPACKLU_DEPRECATED_NO_EXPORT CASADI_LINEARSOLVER_LAPACKLU_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CASADI_LINEARSOLVER_LAPACKLU_NO_DEPRECATED
#endif

#endif
