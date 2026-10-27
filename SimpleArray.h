#pragma once
//=============================================================================
//
// シンプルだけど超有能な動的配列クラフトちゃん [SimpleArray.h]
// Author : 
// 軽量で柔軟な自作動的配列クラス！要素の追加・削除・リサイズもお手の物で、move対応もばっちり
// STLよりちょっぴり素朴だけど、手作業で丁寧にデータを並べてくれる几帳面な子なの~
//
//=============================================================================


template<typename T>
class SimpleArray 
{
private:
    T* array;
    UINT capacity;
    UINT size;

public:
    SimpleArray() : array(nullptr), capacity(0), size(0) {}

    SimpleArray(UINT initialCapacity): size(0)
    {
        capacity = initialCapacity;
        array = new T[capacity];
    }

    SimpleArray(UINT count, const T& defaultValue)
    {
        capacity = count;
        size = count;
        array = new T[capacity];
        for (UINT i = 0; i < size; i++) {
            array[i] = defaultValue;
        }
    }

    SimpleArray(const SimpleArray& other) : array(nullptr), capacity(0), size(0)
    {
        if (other.size > 0) {
            array = new T[other.capacity];
            for (UINT i = 0; i < other.size; i++) {
                array[i] = other.array[i];
            }
            capacity = other.capacity;
            size = other.size;
        }
    }

    SimpleArray& operator=(const SimpleArray& other)
    {
        if (this != &other) 
        {
            delete[] array;
            array = new T[other.capacity];
            for (UINT i = 0; i < other.size; i++) 
            {
                array[i] = other.array[i];
            }
            capacity = other.capacity;
            size = other.size;
        }
        return *this;
    }

    ~SimpleArray() 
    {
        if (array)
        {
            delete[] array;
            array = nullptr;
        }
            
    }

    void push_back(const T& item) 
    {
        if (size >= capacity) 
        {
            int newCapacity = (capacity == 0) ? 4 : capacity * 2;
            T* newArray = new T[newCapacity];
            //for (UINT i = 0; i < size; i++) 
            //{
            //    newArray[i] = array[i];
            //}
            for (UINT i = 0; i < size; i++) {
                newArray[i] = std::move(array[i]);
            }
            delete[] array;
            array = newArray;
            capacity = newCapacity;
        }
        array[size] = item;
        size++;
    }

    void pop_back()
    {
        if (size == 0) return;

        size--;

        // 要素のデストラクタを明示的に呼び出し（非トリビアル型に対応）
        if (!std::is_trivially_destructible<T>::value)
        {
            array[size].~T();
        }
    }

    void push_back(T&& item) {
        if (size >= capacity) {
            int newCapacity = (capacity == 0) ? 4 : capacity * 2;
            T* newArray = new T[newCapacity];
            for (UINT i = 0; i < size; ++i) {
                newArray[i] = std::move(array[i]);
            }
            delete[] array;
            array = newArray;
            capacity = newCapacity;
        }
        array[size] = std::move(item);
        size++;
    }


    const T& front() const
    {
        if (size == 0) 
        {
            return T();
        }
        return array[0];
    }

    const T& back() const
    {
        if (size == 0) 
        {
            return T();
        }
        return array[size - 1]; // Returns a copy of the last element
    }

    void clear()
    {
        //delete[] array;
        //array = nullptr;
        //size = 0;
        //capacity = 0;

        if (!std::is_trivially_destructible<T>::value)
        {
            for (UINT i = 0; i < size; i++)
            {
                array[i].~T();
            }
        }

        size = 0;
    }

    bool empty() const {
        return size == 0;
    }

    UINT getSize() const {
        return size;
    }

	UINT getCapacity() const {
		return capacity;
	}

    void resize(UINT newCapacity) {
        if (newCapacity < size) {
            return;
        }

        T* newArray = new T[newCapacity];
        for (UINT i = 0; i < size; i++) {
            newArray[i] = array[i];
        }
        delete[] array;
        array = newArray;
        capacity = newCapacity;
    }

    void setSize(UINT newSize)
    {
        assert(newSize <= capacity);
        if (newSize < size)
        {
            if (!std::is_trivially_destructible<T>::value)
            {
                for (UINT i = newSize; i < size; ++i)
                    array[i].~T();
            }
        }
        size = newSize;
    }

    void reserve(UINT newCapacity, bool forceShrink = false)
    {
        if (!forceShrink && newCapacity <= capacity) return;

        T* newArray = new T[newCapacity];
        for (UINT i = 0; i < size; ++i)
        {
            newArray[i] = array[i];
        }
        delete[] array;
        array = newArray;
        capacity = newCapacity;
    }

    int find_index(const T& value) const
    {
        for (UINT i = 0; i < size; ++i)
        {
            if (array[i] == value)
                return i;
        }
        return -1;
    }

    void erase(UINT index) 
    {
        if (index >= size) return;

        for (UINT i = index; i < size - 1; i++) 
        {
            array[i] = std::move(array[i + 1]);
        }

        // 最後の要素を明示的にデストラクタ呼び出し（デバッグ安全）
        array[size - 1] = T(); // または std::move(T()) / std::default_constructed<T>()

        size--;
    }

    void shrink_to_fit()
    {
        if (size == 0)
        {
            delete[] array;
            array = nullptr;
            capacity = 0;
        }
        else if (size < capacity)
        {
            T* newArray = new T[size];
            for (int i = 0; i < size; i++)
            {
                newArray[i] = std::move(array[i]);
            }
            delete[] array;
            array = newArray;
            capacity = size;
        }
    }

    T* begin() { return array; }
    T* end() { return array + size; }
    T* data() { return array; }
	const T* data() const { return array; }

    const T* begin() const { return array; }
    const T* end() const { return array + size; }

    T& operator[](UINT index) {
        return array[index];
    }

    const T& operator[](UINT index) const {
        return array[index];
    }
};