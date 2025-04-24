% Main solve predicate
solve :-
    initial_state(InitialState),
    search([ [InitialState, null] ], [], _).

% Initial grid state (5x5)
initial_state([
    [d, '-', p, '-', o],
    ['-', o, '-', '-', p],
    ['-', '-', o, p, '-'],
    [p, o, '-', '-', '-'],
    ['-', '-', p, o, '-']
]).

% Goal condition: Specific final state
goal_condition([
    ['*', '*', '*', '-', o],
    ['-', o, '*', '*', '*'],
    ['-', '-', o, '*', '*'],
    [p, o, '*', '*', '-'],
    ['-', '-', d, o, '-']
]).

% Search implementation (BFS)
search(Open, Closed, _):-
    getState(Open, [CurrentState, Parent], _),
    goal_condition(CurrentState), !,
    write("Drone Route:"), nl, nl,
    printSolution([CurrentState, Parent], Closed),
    write("Final:"), nl,
    printGrid(CurrentState).

search(Open, Closed, Goal):-
    getState(Open, CurrentNode, TmpOpen),
    getAllValidChildren(CurrentNode, TmpOpen, Closed, Children),
    addChildren(Children, TmpOpen, NewOpen),
    append(Closed, [CurrentNode], NewClosed),
    search(NewOpen, NewClosed, Goal).

% Get next valid states
getAllValidChildren(Node, Open, Closed, Children):-
    findall(Next, getNextState(Node, Open, Closed, Next), Children).

getNextState([State,_], Open, Closed, [Next,State]):-
    move(State, Next),
    \+ member([Next,_], Closed),
    \+ member([Next,_], Open).

% BFS implementation
getState([CurrentNode|Rest], CurrentNode, Rest).
addChildren(Children, Open, NewOpen):-
    append(Open, Children, NewOpen).

% Print solution path
printSolution([State, null],_):-
    write("Initial State:"), nl,
    printGrid(State), nl, nl,
    write("Steps:"), nl, nl.

printSolution([State, Parent], Closed):-
    member([Parent, GrandParent], Closed),
    printSolution([Parent, GrandParent], Closed),
    printGrid(State), nl.

% Helper to print grid
printGrid([]).
printGrid([H|T]) :-
    write('    '),  % Add indentation
    write(H), nl,
    printGrid(T).

% Movement predicates
move(State, Next):-
    left(State, Next); right(State, Next); up(State, Next); down(State, Next).

% Find drone position
find_drone(State, (Row, Col)) :-
    nth0(Row, State, RowList),
    nth0(Col, RowList, d).

% Left move
left(State, Next):-
    find_drone(State, (R, C)),
    C > 0,
    NewC is C - 1,
    nth0(R, State, Row),
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Right move
right(State, Next):-
    find_drone(State, (R, C)),
    nth0(R, State, Row),
    length(Row, Cols),
    C < Cols - 1,
    NewC is C + 1,
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Up move
up(State, Next):-
    find_drone(State, (R, C)),
    R > 0,
    NewR is R - 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% Down move
down(State, Next):-
    find_drone(State, (R, C)),
    length(State, Rows),
    R < Rows - 1,
    NewR is R + 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% Mark positions with * for horizontal moves
mark_position(State, R, C, NewC, Next) :-
    substitute_in_grid(State, R, C, '*', TempState),
    substitute_in_grid(TempState, R, NewC, 'd', Next).

% Mark positions with * for vertical moves
mark_position_row(State, R, C, NewR, Next) :-
    substitute_in_grid(State, R, C, '*', TempState),
    substitute_in_grid(TempState, NewR, C, 'd', Next).

% Helper to substitute an element in the grid
substitute_in_grid(Grid, Row, Col, NewElement, NewGrid) :-
    nth0(Row, Grid, OldRow),
    substitute_in_row(OldRow, Col, NewElement, NewRow),
    substitute(OldRow, Grid, NewRow, NewGrid).

% Helper to substitute in a row
substitute_in_row(Row, 0, NewElement, [NewElement|Rest]) :-
    Row = [_|Rest].
substitute_in_row([H|T], Col, NewElement, [H|NewT]) :-
    Col > 0,
    Col1 is Col - 1,
    substitute_in_row(T, Col1, NewElement, NewT).

% General substitute predicate
substitute(Element, [Element|T], NewElement, [NewElement|T]) :- !.
substitute(Element, [H|T], NewElement, [H|NewT]) :-
    substitute(Element, T, NewElement, NewT).