/* Copyright (C) 2012 mbed.org, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CIRCBUFFER_H_
#define CIRCBUFFER_H_

template <class T>
class CircBuffer 
{
public:
    CircBuffer(int length)
	{
        write = 0;
        read = 0;
        size = length + 1;
        buf = (T *)malloc(size * sizeof(T));
    };
	
	bool getWrite() 
	{
		return write;
	};

    bool isFull() 
	{
        return (((write + 1) % size) == read);
    };

    bool isEmpty()
	{
        return (read == write);
    };

    void queue(T k) 
	{
        if (isFull()) 
        {
            read++;
            read %= size;
        }
        buf[write++] = k;
        write %= size;
    }
    
    void flush() 
    {
        read = 0;
        write = 0;
    }
    
    uint32_t available() 
    {
        return (write >= read) ? write - read : size - read + write;
    };
	
	uint32_t getLength(uint32_t indx) 
	{
		return (indx >= read) ? indx - read + 1 : size - read + indx + 1;
	};
	
	T getAt(uint32_t i)
	{
		uint32_t curr = read + i;
		curr %= size;		
		return buf[curr];
	};
	
	bool isValid(uint32_t i)
	{
		if (isEmpty())
			return false;
		
		uint32_t curr = read + i;
		curr %= size;	
		if (write < read)
			return curr >= read || curr < write;
		
		return curr < write;
	};

    bool dequeue(T * c) 
    {
        bool empty = isEmpty();
        if (!empty)
        {
            *c = buf[read++];
            read %= size;
        }
        return(!empty);
    };
	
	uint32_t find(T& c)
    {
		if (isEmpty())
			return -1;
		
	    uint32_t end = write;	// Gaurd against volatile write
	    uint32_t curr = read;
	    T x;
	    
	    while (curr != end)
	    {
		    x = buf[curr];
		    if (c == x)
			    return curr;
		    
		    ++curr;
		    if (curr == size)
			    curr = 0;
		}
		
		return -1;
	};
	
	uint32_t find(const T* s, size_t arraySz)
	{
		if (isEmpty() || arraySz == 0 || available() < arraySz)
			return -1;
		
		uint32_t end = write;	// Gaurd against volatile write
		uint32_t curr = read;
		uint32_t last = 0;
		uint32_t indx = 0;
	    
		while (curr != end)
		{
			bool found = true;
			for (size_t i = 0; i < arraySz; ++i)
			{
				if (s[i] != buf[curr + i])
				{
					found = false;
					break;
				}
			}
			
			if (found)
				return curr;
			
			if (curr < size - 1)
				++curr;
			else
				curr = 0;
		}
		
		return -1;
	};

private:
    volatile uint32_t write;
    volatile uint32_t read;
    uint32_t size;
    T * buf;
};

#endif
