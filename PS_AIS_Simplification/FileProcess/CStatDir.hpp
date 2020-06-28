#pragma once
#include "../stdafx.h"
#include "stdlib.h"
#include "direct.h"
#include "string.h"
#include "io.h"
#include "stdio.h" 
#include "iostream"
#include "CBrowseDir.h"

class CStatDir :public CBrowseDir
{
protected:
	int m_nFileCount;   //�����ļ�����
	int m_nSubdirCount; //������Ŀ¼����

public:
	//ȱʡ������
	CStatDir()
	{
		//��ʼ�����ݳ�Աm_nFileCount��m_nSubdirCount
		m_nFileCount = m_nSubdirCount = 0;
		
	}

	//�����ļ�����
	int GetFileCount()
	{
		return m_nFileCount;
	}

	//������Ŀ¼����
	int GetSubdirCount()
	{
		//��Ϊ�����ʼĿ¼ʱ��Ҳ����ú���ProcessDir��
		//���Լ�1�������������Ŀ¼������
		return m_nSubdirCount - 1;
	}

protected:
	//��д�麯��ProcessFile��ÿ����һ�Σ��ļ�������1
	virtual bool ProcessFile(const char *filename)
	{
		m_nFileCount++;
		return CBrowseDir::ProcessFile(filename);
	}

	//��д�麯��ProcessDir��ÿ����һ�Σ���Ŀ¼������1
	virtual void ProcessDir
		(const char *currentdir, const char *parentdir)
	{
		m_nSubdirCount++;
		CBrowseDir::ProcessDir(currentdir, parentdir);
	}
};