        title   "general.asm"

clc     MACRO
        bcf    STATUS,C
        ENDM

sec     MACRO
        bsf    STATUS,C
        ENDM

skipz   MACRO
        btfss  STATUS,Z
        ENDM

skipnz  MACRO
        btfsc  STATUS,Z
        ENDM

skipnc  MACRO
        btfsc  STATUS,C
        ENDM

skipc   MACRO
        btfss  STATUS,C
        ENDM

beq     MACRO  _Label_
        btfsc  STATUS,Z
        goto   _Label_
        ENDM

bne     MACRO  _Label_
        btfss  STATUS,Z
        goto   _Label_
        ENDM

bcc     MACRO  _Label_
        btfss  STATUS,C
        goto   _Label_
        ENDM

bcs     MACRO  _Label_
        btfsc  STATUS,C
        goto   _Label_
        ENDM

ENI     MACRO
        bsf    INTCON,GIE
        ENDM

DII     MACRO
        bcf    INTCON,GIE
        
        ENDM


RESPCH	MACRO
#if high $ <= 7
	clrf	PCLATH
#else
        movlw   high $
        movwf   PCLATH
#endif
	ENDM

SETPCH	MACRO	_Label_
#if high _Label_ < 8
	clrf	PCLATH
#else
        movlw   high _Label_
        movwf   PCLATH
#endif
	ENDM


; added for 18f2520
#define rrf rrncf
#define rlf rlncf


