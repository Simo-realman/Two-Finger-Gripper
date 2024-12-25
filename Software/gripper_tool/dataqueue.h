#ifndef DATAQUEUE_H
#define DATAQUEUE_H
#include <QObject>
#include <QMutex>
#include <stdlib.h>
#include <malloc.h>

#define QUEUE_SIZE    500     //CANFD队列容量

typedef struct {
    quint8 id;
    quint8 cmd;
    quint16 len;
    quint8 data[100];
}queue_data;

typedef struct queue
{
    queue_data* pBase;
    int front;
    int rear;
    int maxsize;
}QUEUE;

class DataQueue
{
public:
    DataQueue();
public:
    static DataQueue* GetInstance();
    static void ExitInstance();

    int getLength();
    void Clear_queue();
    unsigned char Dequeue(queue_data *val);
    unsigned char Enqueue(queue_data *val);
    unsigned char EmptyQueue();
    unsigned char FullQueue();
    unsigned char CreateQueue(int maxsize);
private:
    static DataQueue *m_pDataQueue;
    //CANFD下发队列
    QUEUE CANFD_buffer;
};

#define DATAQUEUE DataQueue::GetInstance()

#endif // DATAQUEUE_H
