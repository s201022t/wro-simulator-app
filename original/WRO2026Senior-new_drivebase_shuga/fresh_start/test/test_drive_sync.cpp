#include "drive_sync.h"
using drive_sync::apply;

constexpr auto leftAhead=apply(100,100,100,100,1000,1000,600,500,2,true);
static_assert(leftAhead.left<100 && leftAhead.right==100,"Slow only the leading left wheel");
constexpr auto rightAhead=apply(100,100,100,100,1000,1000,500,600,2,true);
static_assert(rightAhead.left==100 && rightAhead.right==leftAhead.left,"Symmetric steering");
constexpr auto arc=apply(100,50,100,50,1000,500,400,200,2,true);
static_assert(arc.left==100 && arc.right==50,"Equal progress ratios need no steering");
constexpr auto bounded=apply(10,20,100,100,1000,1000,900,0,100,true);
static_assert(bounded.left==0 && bounded.right==20,"Steering must not reverse or boost a wheel");
constexpr auto pivot=apply(100,0,100,0,1000,0,500,0,2,true);
static_assert(pivot.left==100 && pivot.right==0,"Single-wheel motion bypasses sync");
constexpr auto terminal=apply(20,30,100,100,1000,1000,999,900,2,false);
static_assert(terminal.left==20 && terminal.right==30,"Endpoint correction stays independent");
constexpr auto reverseCorrection=apply(-10,20,100,100,1000,1000,600,500,2,true);
static_assert(reverseCorrection.left==-10 && reverseCorrection.right==20,"Do not mix reverse position correction");
constexpr auto disabled=apply(100,100,100,100,1000,1000,600,500,0,true);
static_assert(disabled.left==100 && disabled.right==100,"Zero gain disables sync");
// Forward, reverse and counter-rotation use the same normalized progress;
// drive() applies each requested wheel direction after speed/PWM control.
