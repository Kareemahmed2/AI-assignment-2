% Main solve predicate
solve :-
    initial_state(InitialState),
    goal_state(GoalState),
    calculateH(InitialState, GoalState, H),
    search([ [InitialState, null, 0, H, H] ], [], GoalState).

% ========== STATE DEFINITIONS ==========
initial_state([
    [d, '-', p, '-', o],
    ['-', o, '-', '-', p],
    ['-', '-', o, p, '-'],
    [p, o, '-', '-', '-'],
    ['-', '-', p, o, '-']
]).

% Goal state 
goal_state(GoalState) :- 
    initial_state(GoalState). 

% ========== SEARCH CORE ==========
search(Open, Closed, Goal):-
    getBestState(Open, [CurrentState, Parent, G, H, F], TmpOpen),
    goal_condition(CurrentState, Goal), !,
    write("Optimal Drone Route Found!"), nl, nl,
    printSolution([CurrentState, Parent, G, H, F], Closed),
    write("Final State:"), nl,
    printGrid(CurrentState).

search(Open, Closed, Goal):-
    getBestState(Open, CurrentNode, TmpOpen),
    getAllValidChildren(CurrentNode, TmpOpen, Closed, Goal, Children),
    addChildren(Children, TmpOpen, NewOpen),
    append(Closed, [CurrentNode], NewClosed),
    search(NewOpen, NewClosed, Goal).

% ========== GOAL CHECKING ==========
goal_condition(State, _) :-
    flatten(State, FlatState),
    \+ member(p, FlatState).

% ========== HEURISTIC FUNCTION ==========
calculateH(State, _, H):-
    flatten(State, FlatState),
    findall(X, (member(X, FlatState), X == p), Ps),
    length(Ps, H).

% ========== NODE SELECTION ==========
getBestState(Open, BestChild, Rest):-
    findMin(Open, BestChild),
    delete(Open, BestChild, Rest).

findMin([X], X):- !.
findMin([Head|T], Min):-
    findMin(T, TmpMin),
    Head = [_, _, _, _, HeadF],
    TmpMin = [_, _, _, _, TmpF],
    (TmpF < HeadF -> Min = TmpMin ; Min = Head).

% ========== CHILD NODE GENERATION ==========
getAllValidChildren(Node, Open, Closed, Goal, Children):-
    findall(Next, getNextState(Node, Open, Closed, Goal, Next), Children).


getNextState([State, Parent, G, _, _], Open, Closed, Goal, [Next, State, NewG, NewH, NewF]):-
    move(State, Next, MoveCost),
    calculateH(Next, Goal, NewH),
    NewG is G + MoveCost,
    NewF is NewG + NewH,
    (not(member([Next, _, _, _, _], Open)) ; memberButBetter(Next, Open, NewF)),
    (not(member([Next, _, _, _, _], Closed)) ; memberButBetter(Next, Closed, NewF)).

memberButBetter(Next, List, NewF):-
    findall(F, member([Next, _, _, _, F], List), Numbers),
    (Numbers = [] -> true ; min_list(Numbers, MinOldF), MinOldF > NewF).

addChildren(Children, Open, NewOpen):-
    append(Open, Children, NewOpen).

% ========== SOLUTION PRINTING ==========
printSolution([State, null, G, H, F], _):-
    write("Initial State (g="), write(G), write(", h="), write(H), write(", f="), write(F), write("):"), nl,
    printGrid(State), nl, nl,
    write("Steps:"), nl, nl.

printSolution([State, Parent, G, H, F], Closed):-
    member([Parent, GrandParent, PrevG, PH, PF], Closed),
    printSolution([Parent, GrandParent, PrevG, PH, PF], Closed),
    write("Move (g="), write(G), write(", h="), write(H), write(", f="), write(F), write("):"), nl,
    printGrid(State), nl.

printGrid([]).
printGrid([H|T]) :-
    write('    '),
    write(H), nl,
    printGrid(T).

% ========== MOVEMENT RULES ==========
move(State, Next, 1):-
    left(State, Next); right(State, Next); 
    up(State, Next); down(State, Next).

find_drone(State, (Row, Col)) :-
    nth0(Row, State, RowList),
    nth0(Col, RowList, d).

% Left movement
left(State, Next):-
    find_drone(State, (R, C)),
    C > 0,
    NewC is C - 1,
    nth0(R, State, Row),
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Right movement
right(State, Next):-
    find_drone(State, (R, C)),
    nth0(R, State, Row),
    length(Row, Cols),
    C < Cols - 1,
    NewC is C + 1,
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Up movement
up(State, Next):-
    find_drone(State, (R, C)),
    R > 0,
    NewR is R - 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% Down movement
down(State, Next):-
    find_drone(State, (R, C)),
    length(State, Rows),
    R < Rows - 1,
    NewR is R + 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% ========== GRID MODIFICATION ==========
mark_position(State, R, C, NewC, Next) :-
    substitute_in_grid(State, R, C, '-', TempState),
    (nth0(R, TempState, TempRow), nth0(NewC, TempRow, p) -> 
        substitute_in_grid(TempState, R, NewC, 'd', Next) ;
        substitute_in_grid(TempState, R, NewC, 'd', Next)).

mark_position_row(State, R, C, NewR, Next) :-
    substitute_in_grid(State, R, C, '-', TempState),
    (nth0(NewR, TempState, TempRow), nth0(C, TempRow, p) -> 
        substitute_in_grid(TempState, NewR, C, 'd', Next) ;
        substitute_in_grid(TempState, NewR, C, 'd', Next)).

% Improved grid manipulation predicates
substitute_in_grid(Grid, Row, Col, NewElement, NewGrid) :-
    length(Grid, NumRows),
    Row < NumRows,
    nth0(Row, Grid, OldRow),
    substitute_in_row(OldRow, Col, NewElement, NewRow),
    replace_nth0(Row, Grid, NewRow, NewGrid).

substitute_in_row(Row, Col, NewElement, NewRow) :-
    length(Row, NumCols),
    Col < NumCols,
    replace_nth0(Col, Row, NewElement, NewRow).

% Helper predicate to replace element at index N in a list
replace_nth0(0, [_|T], X, [X|T]) :- !.
replace_nth0(N, [H|T], X, [H|R]) :-
    N > 0,
    N1 is N - 1,
    replace_nth0(N1, T, X, R).