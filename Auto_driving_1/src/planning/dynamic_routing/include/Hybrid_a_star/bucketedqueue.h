/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

#define MAXDIST 1000
#define RESERVE 64

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include "point.h"

//! Priority queue for integer coordinates with squared distances as priority.
/** A priority queue that uses buckets to group elements with the same priority.
    The individual buckets are unsorted, which increases efficiency if these groups are large.
    The elements are assumed to be integer coordinates, and the priorities are assumed
    to be squared euclidean distances (integers).
*/
class BucketPrioQueue
{

public:
    //! Standard constructor
    /** Standard constructor. When called for the first time it creates a look up table
        that maps square distanes to bucket numbers, which might take some time...
    */
    BucketPrioQueue();
    //! Checks whether the Queue is empty
    bool empty() const;
    //! push an element
    void push(int prio, INTPOINT t);
    //! return and pop the element with the lowest squared distance */
    INTPOINT pop();

private:
    static void initSqrIndices();
    static std::vector<int> sqrIndices;
    static int numBuckets;
    int count;
    int nextBucket;

    std::vector<std::queue<INTPOINT>> buckets;
};
#endif
