#include "dataqueue.h"

QMutex m_queueMutex;
DataQueue* DataQueue::m_pDataQueue = NULL;

DataQueue::DataQueue()
{

}

DataQueue *DataQueue::GetInstance()
{
    if(NULL == m_pDataQueue) {
        m_queueMutex.lock();
        if(NULL == m_pDataQueue) {
            m_pDataQueue = new DataQueue();
        }
        m_queueMutex.unlock();
    }
    return m_pDataQueue;
}

void DataQueue::ExitInstance()
{
    if (m_pDataQueue != NULL)
    {
        m_queueMutex.lock();
        delete m_pDataQueue;
        m_pDataQueue = NULL;
        m_queueMutex.unlock();
    }
}

unsigned char DataQueue::CreateQueue(int maxsize)
{
    CANFD_buffer.pBase = (queue_data *)malloc(sizeof(queue_data)*maxsize);
    if(NULL==CANFD_buffer.pBase)
    {
        return 0;
    }
    CANFD_buffer.front=0;
    CANFD_buffer.rear=0;
    CANFD_buffer.maxsize=maxsize;

    return 1;
}
/***********************************************
Function: Print the stack element;
************************************************/
//队列已满，返回1
unsigned char DataQueue::FullQueue()
{
    if(CANFD_buffer.front==(CANFD_buffer.rear+1)%CANFD_buffer.maxsize)
        return 1;
    else
        return 0;
}

//空，返回1；不为空，返回0
unsigned char DataQueue::EmptyQueue()
{
    if(CANFD_buffer.front==CANFD_buffer.rear)
        return 1;
    else
        return 0;
}

unsigned char DataQueue::Enqueue( queue_data *val)
{
    int i = 0;
    if(FullQueue())
        return 0;
    else
    {
        CANFD_buffer.pBase[CANFD_buffer.rear].id = val->id;
        CANFD_buffer.pBase[CANFD_buffer.rear].cmd = val->cmd;
        CANFD_buffer.pBase[CANFD_buffer.rear].len = val->len;
        for(i=0; i < val->len; i++)
            CANFD_buffer.pBase[CANFD_buffer.rear].data[i] = val->data[i];

        CANFD_buffer.rear=(CANFD_buffer.rear+1)%CANFD_buffer.maxsize;
        return 1;
    }
}

unsigned char DataQueue::Dequeue(queue_data *val)
{
    int i = 0;
    if(EmptyQueue())
    {
        return 0;
    }
    else
    {
        val->id = CANFD_buffer.pBase[CANFD_buffer.front].id;
        val->cmd = CANFD_buffer.pBase[CANFD_buffer.front].cmd;
        val->len = CANFD_buffer.pBase[CANFD_buffer.front].len;
        for(i=0;i<val->len;i++)
            val->data[i] = CANFD_buffer.pBase[CANFD_buffer.front].data[i];

        CANFD_buffer.front=(CANFD_buffer.front+1)%CANFD_buffer.maxsize;
        return 1;
    }
}

void DataQueue::Clear_queue()
{
    CANFD_buffer.front = CANFD_buffer.rear;
}

int DataQueue::getLength()
{

    return (CANFD_buffer.rear - CANFD_buffer.front + QUEUE_SIZE) % QUEUE_SIZE;
}
