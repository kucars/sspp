/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "llist.h"

namespace SSPP
{

Node * LList::getHead()
{
    return this->Start;
}

void LList::next()
{
    Start = Start->next;
    if(Start != NULL)
        Start->prev = NULL;
}

void LList::prev()
{
    Start = Start->prev;
}

Node * LList::find(Node * q)
{
    Node *p = Start;
    while (p)
    {
        if(*q==*p)
        {
            return p;
        }
        p = p->next;
    }
    return NULL;
}

LList::LList()
{
    Start = NULL;
}

LList::~LList()
{
    Node *p;
    while (Start != NULL)
    {
        p = Start->next;
        delete Start;
        Start = p;
    }
}

void LList::free()
{
    Node *p;
    while (Start != NULL)
    {
        p = Start->next;
        delete Start;
        Start = p;
    }
}

void LList::add(Node * curChild, bool ascending)
{
    Node * p,* q ;
    bool condition;
    p = this->Start;
    q = p;
    // now insert the child into the open list according to the f value
    while (p)
    {
        //cost function
        if(ascending)
            condition = (p->f_value >= curChild->f_value);
        //reward function
        else
            condition = (p->f_value <= curChild->f_value);
        if(condition)
        {
            // test head of the list case
            if (p == Start)
                Start = curChild;
            curChild->next = p;
            curChild->prev = p->prev;
            p->prev = curChild;
            if (curChild->prev)
                (curChild->prev)->next = curChild;
            break;
        }
        q = p;
        p = p->next;
    }
    if (p == NULL)
    {
        if (q != NULL) // insert at the end
        {
            q->next = curChild;
            curChild->prev = q;
            curChild->next = NULL;
        }
        else	      // insert at the beginning
        {
            Start = curChild;
            curChild->prev = NULL;
        }
    }
}

bool LList::remove(Node *q)
{
    Node *p;
    p = Start;
    while (p)
    {
        if(*q==*p)
        {
            if (p->prev != NULL)
                (p->prev)->next = p->next;
            if (p->next != NULL)
                (p->next)->prev = p->prev;
            if (p == Start)
                Start = p->next;
            delete p;
            return 1;
        }
        p = p->next;
    }
    return 0;
}

void LList::print()
{
    Node *p;
    int i=0;
    p = Start;
    while(p)
    {
        cout<<"\n Node["<<++i<<"] X="<<p->pose.p.position.x<<" Y="<<p->pose.p.position.y<<" Z="<<p->pose.p.position.z<<" F value="<<p->f_value;
        p = p->next;
    }
}
}
