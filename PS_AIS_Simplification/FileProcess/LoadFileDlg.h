#pragma once

#include <windows.h>
#include <stdio.h>
#include <commdlg.h>

static OPENFILENAME ofn ;

// Init the file dlg
void PopFileInitialize (HWND hwnd);

// Open the file dlg
BOOL PopFileOpenDlg (HWND hwnd, PTSTR pstrFileName, PTSTR pstrTitleName);