#ifndef MSG_CHANNEL_HPP
#define MSG_CHANNEL_HPP

#include <queue>
#include <pthread.h>

namespace frc973 {

// Channel is a bounded concurrent FIFO queue.  It's roughly equivalent
// to a buffered channel in Go.
template <class T> class Channel {
  public:
    Channel(int size) : m(PTHREAD_MUTEX_INITIALIZER), cond(PTHREAD_COND_INITIALIZER), sz(size) {
      // TODO: prealloc space in queue?
    }
 
    ~Channel() {
      pthread_cond_destroy(&cond);
      pthread_mutex_destroy(&m);
    }
 
    bool send(const T &val) {
      bool success = false;
      pthread_mutex_lock(&m);
      // TODO: this just drops values that overflow without blocking.
      if (q.size() < sz) {
        q.push(val);
        success = true;
      }
      pthread_cond_signal(&cond);
      pthread_mutex_unlock(&m);
      return success;
    }

    // Empties the queue before appending the value
    // ensures the newest value is the one read first
    bool sendNewest(const T &val) {
        pthread_mutex_lock(&m);
        while (!q.empty()) {
            q.pop();
            pthread_cond_wait(&cond, &m);
        }
        q.push(val);
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&m);
        return true;
    }

    int size() {
    	pthread_mutex_lock(&m);
    	int sz = q.size();
    	pthread_cond_signal(&cond);
    	pthread_mutex_unlock(&m);
    	return sz;
    }
 
    T recv() {
      pthread_mutex_lock(&m);
      while (q.empty()) {
        pthread_cond_wait(&cond, &m);
      }
      T val = q.front();
      q.pop();
      pthread_mutex_unlock(&m);
      return val;
    }

    T recvNonBlock() {
      pthread_mutex_lock(&m);
      if (q.empty()) {
          return NULL;
      }
      T val = q.front();
      q.pop();
      pthread_mutex_unlock(&m);
      return val;
    }
 
  private:
    pthread_mutex_t m;
    pthread_cond_t cond;
    std::queue<T> q;
    unsigned int sz;
};
    
}

#endif
